#! /usr/bin/env python

import rospy
import tf_conversions

import actionlib

import path_tracking.msg

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import visualization_msgs.msg

import tf  # tf is deprecated: use tf2
import tf2_ros

from path_tracking.geometry2d import *
import path_tracking.kinematics
import math
import numpy as np

import path_tracking



# TODO: this function should be moved to path_tracking/kinematics.py
def v_braking(v_max, a_max, jerk):
    """
    The function returns a function that takes a distance to goal as input
    and returns a velocity (a velocity that allows stopping at the goal 
    satisfying v_max, a_max and jerk
    """

    solution = path_tracking.kinematics.plan_velocity_change( a_max=a_max, jerk=jerk, v_0=0, v_g=v_max, a_0=0, a_g=0, debug=False )
    (dt1, dt2, dt3), (dx1, dx2, dx3), (v1, v2), jerks, velocity_change_distance = solution

    def v(t):
        if t <= dt1:
            v_ = jerk * t**2 / 2.
        elif t <= dt1+dt2:
            v_ = v1 + a_max * (t - dt1)
        else: #
            v_ = v_max - jerk * (dt1+dt2+dt3 - t)**2 / 2.
        return v_
    def x(t):
        if t <= dt1:
            x_ = jerk * t**3 / 6.
        elif t <= dt1+dt2:
            x_ = dx1 + v1 * (t - dt1) + a_max/2. * (t - dt1)**2
        else: #
            x_ = (dx1+dx2+dx3) - v_max * (dt1+dt2+dt3-t) + jerk * (dt1+dt2+dt3-t)**3 / 6.
        return x_

    t  = np.linspace(  0, dt1+dt2+dt3, 100)
    v = [ v(t_) for t_ in t ]
    x = [ x(t_) for t_ in t ]

    from scipy import interpolate
    v_braking = interpolate.interp1d( x, v )
    return v_braking # v_braking(x)



class PursuitPathTrackerServer(path_tracking.Server):


    def __init__(self):
        super(PursuitPathTrackerServer, self).__init__()

        # create publishers for dbg info
        self._pub_pursuit_marker = rospy.Publisher('~pursuit_marker', visualization_msgs.msg.Marker, queue_size=1)

        # dynamic_reconfigure
        self._config = None
        def _config_cb(config, level):
            print('config updated :)', level)
            print(config)
            self._config = config
            # recalculate breaking distance
            v_max = self._config['v_max']
            a_max = self._config['a_max']
            jerk  = self._config['jerk']
            _, velocity_change_distance = path_tracking.kinematics.plan_velocity_change_1dof( a_max=a_max, v_0=v_max, v_g=0 )
            print('velocity_change_distance:', velocity_change_distance)
            #
            return config
        import dynamic_reconfigure.server
        from upnabot_path_tracking.cfg import PursuitPathTrackingConfig
        srv = dynamic_reconfigure.server.Server(PursuitPathTrackingConfig, _config_cb)

        self._pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        # dbg
        self._pub_ang_vel_1 = rospy.Publisher('/ang_vel_1', std_msgs.msg.Float32, queue_size=1)
        self._pub_ang_vel_2 = rospy.Publisher('/ang_vel_2', std_msgs.msg.Float32, queue_size=1)


    def _cmd_loop_iter(self, t_now, dt):

        # using config as a local reference avoids concurrency problems caused by the config_cb
        config = self._config
        if not config:
            rospy.logwarn('dynamic config no ready yet => ignoring odom_cb')
            assert False # is this block neccesary?
            return

        # using path as a local reference avoids concurrency problems with execute_cb()
        path = self._path
        if path is None: # not tracking any path currently
            # command robot stopped
            twist = geometry_msgs.msg.Twist()
            #twist.linear.x = velocity
            #twist.angular.z = angular_vel
            self._pub.publish(twist)
            #
            return
        path_frame = self._path_frame # concurrency problems?
        reverse    = self._reverse    # concurrency problems?

        # transform robot pose (in odom frame) to path frame coords
        #to_frame = path_frame
        #from_frame = 'base_link'
        from_frame = path_frame  # TODO: TIP: it seems that from and to are just the opposite
        to_frame = 'base_link'
        try:
            #robot_position, robot_orientation = self._tf_listener.lookupTransform(from_frame, to_frame, rospy.Time()) # last common tf
            #trans = self._tfBuffer.lookup_transform(from_frame, to_frame, rospy.Time(), rospy.Duration(0.5))
            trans = self._tfBuffer.lookup_transform(from_frame, to_frame, rospy.Time())
            robot_position = trans.transform.translation
            robot_position = [robot_position.x, robot_position.y, robot_position.z]
            robot_orientation = trans.transform.rotation
            robot_orientation = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
        except tf2_ros.LookupException as unused_e:
            rospy.logwarn('could not lookuptf from %s frame to %s frame (LE)' % (from_frame, to_frame))
            return
        except tf2_ros.ConnectivityException as unused_e:
            rospy.logwarn('could not lookuptf from %s frame to %s frame (CE)' % (from_frame, to_frame))
            return
        except tf2_ros.ExtrapolationException as unused_e:
            rospy.logwarn('could not lookuptf from %s frame to %s frame (EE)' % (from_frame, to_frame))
            # TODO: take security actions
            return
        #
        robot_position = Point( robot_position[0], robot_position[1] )
        #
        q = robot_orientation
        #q = ( q.x, q.y, q.z, q.w )
        robot_heading = tf_conversions.transformations.euler_from_quaternion(q)[2]
        robot_heading_vec = Vector( math.cos(robot_heading), math.sin(robot_heading) )

        # pure-pursuit
        # find nearest point in the path
        ppoint, path_heading_vec, icr = path.project_point(robot_position)
        path_heading_vec = path_heading_vec.normalize()
        path_heading = path_heading_vec.heading()
        # calc pursuit point
        pursuit_dist = self._config['pursuit_distance']
        pursuit_point = Point( ppoint.x + pursuit_dist * path_heading_vec.x, ppoint.y + pursuit_dist * path_heading_vec.y )

        #
        dist2goal = path.distance_to_goal(robot_position)

        # dbg: visualize pursuit_point
        # publish pursuit_point frame
        br = tf.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = t_now
        t.header.frame_id = path_frame
        t.child_frame_id  = 'pursuit'
        t.transform.translation.x = pursuit_point.x
        t.transform.translation.y = pursuit_point.y
        t.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, path_heading)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransformMessage(t)
        # publish rviz marker
        marker = visualization_msgs.msg.Marker()
        marker.id = 0
        marker.header.stamp = t_now
        marker.header.frame_id = path_frame
        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.ns = "pursuit"
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(1)
        marker.pose.position.x = pursuit_point.x
        marker.pose.position.y = pursuit_point.y
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        self._pub_pursuit_marker.publish(marker)

        # heading and cross track error
        # heading err
        if reverse:
            heading_err = ( robot_heading + math.pi ) - path_heading
        else:
            heading_err = robot_heading - path_heading
        while heading_err < -math.pi:
             heading_err = heading_err + 2 * math.pi
        while heading_err > +math.pi:
             heading_err = heading_err - 2 * math.pi
        # cte (cross track error)
        cte = robot_position.distance(ppoint) * np.sign(path_heading_vec.cross(Vector(ppoint.x-robot_position.x, ppoint.y-robot_position.y))) # is the sign OK?
        # publish
        self._pub_heading_err.publish(std_msgs.msg.Float32(heading_err))
        self._pub_cte.publish(std_msgs.msg.Float32(cte))

        # velocity control
        v_max = self._config['v_max']
        a_max = self._config['a_max']
        jerk  = self._config['jerk']

        #
        if self._tracking_done is None: # tracking not started
            self._tracking_done = False # set tracking to in-progress
            #

            # Estimate the distance required to speed up from 0 to v_max and,
            # if such distance is greater than half the distance to goal,
            # iteratively reduce v_max until the condition is satisfied
            #
            while True:
                solution = path_tracking.kinematics.plan_velocity_change( a_max=a_max, jerk=jerk, v_0=0, v_g=v_max, a_0=0, a_g=0, debug=False )
                (dt1, dt2, dt3), (dx1, dx2, dx3), (v1, v2), jerks, velocity_change_distance = solution
                velocity_change_distance = dx1 + dx2 + dx3
                print('d2g=%f, velocity_change_distance=%f, v_max=%f' % (dist2goal, velocity_change_distance, v_max))
                if 2 * velocity_change_distance <= dist2goal:
                    break
                else:
                    v_max = 0.7 * v_max
            self._v_max = v_max

            #
            self._brake_distance = velocity_change_distance # when distance to goal is smaller than _brake_distance velocity starts slowing down
            self._max_vel_at_max_acc = v2 # when this velocity is reached acceleration starts slowing down
            self._acceleration = 0 # current acceleration
            self._velocity = 0.    # current velocity
            self._v_braking = v_braking(v_max, a_max, jerk)
            #
        else:
            v_max = self._v_max


        # speed control
        if dist2goal <= self._brake_distance:
            # arriving to goal => deceleration phase
            a = 0 # not used in this phase
            velocity = self._v_braking( dist2goal )
            velocity = min( velocity, self._velocity )
        else:
            # far from goal => speed up to achieve target velocity (v_max)
            a = self._acceleration
            if self._velocity >= self._max_vel_at_max_acc:
                a -= jerk * dt
                a = max( a, 0 )
            elif a <= a_max:
                a += jerk * dt
                a = min( a, a_max )
            velocity = self._velocity + a * dt
            velocity = min( velocity, v_max )
        # goal reached?
        if dist2goal <= 0 or self._tracking_done is True: # latch to goal
            velocity = 0
            if self._tracking_done is False:
                self._tracking_done = True
        #
        self._velocity = velocity
        self._acceleration = a

        # calculate the angular velocity required to follow the path
        # two angular velocities will be calculated
        #   angular_vel_1: the angular velocity to go to the pursuit point (see curve_radius_to_point.png)
        #   angular_vel_2: the angular velocity to blindly follow the path curvature at its nearest point
        # angular_vel_1 calculation
        delta_y = Line(robot_position,robot_heading_vec).distance(pursuit_point)
        delta_x = Line(robot_position,robot_heading_vec.rot90()).distance(pursuit_point)
        ca = math.cos(2*math.atan2(delta_y,delta_x))
        if 1 - ca < 1e-9:
            # R_1 = np.Infinity
            angular_vel_1 = 0
        else:
            R_1 = delta_y / (1 - ca)
            direction_1 = np.sign(robot_heading_vec.cross(Vector(pursuit_point.x-robot_position.x, pursuit_point.y-robot_position.y)))
            angular_vel_1 = velocity / R_1 * direction_1
        # angular_vel_2 calculation
        if icr.x is np.Infinity:
            # R_2 = np.Infinity
            angular_vel_2 = 0
        else:
            R_2 = icr.distance(ppoint)
            direction_2 = np.sign(path_heading_vec.cross(Vector(icr.x-ppoint.x, icr.y-ppoint.y)))
            if self._reverse:
                direction_2 = - direction_2
            angular_vel_2 = velocity / R_2 * direction_2
        # combine angular_vel_1 and angular_vel_2
        d2path = robot_position.distance(ppoint)
        # only when the robot is near the path follow its curvature (avoiding discontinuities)
        if d2path <= pursuit_dist:
            angular_vel = angular_vel_1 + angular_vel_2
        else:
            angular_vel = angular_vel_1 + angular_vel_2 * ( pursuit_dist / d2path )**2

        # dbg
        self._pub_ang_vel_1.publish(std_msgs.msg.Float32(angular_vel_1))
        self._pub_ang_vel_2.publish(std_msgs.msg.Float32(angular_vel_2))

        # publish twist msg
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = velocity
        twist.angular.z = angular_vel
        if self._reverse:
            twist.linear.x  = - twist.linear.x
            twist.angular.z = - twist.angular.z # this should not be required, but kobuki does
        self._pub.publish(twist)

        #rospy.loginfo('---- odom (end--)')



def main():
    server = PursuitPathTrackerServer()
    server.run()


if __name__ == '__main__':
    main()
