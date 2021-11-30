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

            # dbg: recalculate breaking distance
            v_max = self._config['v_max']
            a_max = self._config['a_max']
            _, velocity_change_distance = path_tracking.kinematics.plan_velocity_change_1dof( a_max=a_max, v_0=v_max, v_g=0 )
            print('velocity_change_distance:', velocity_change_distance)

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
        # TODO: not used!!
        config = self._config
        if not config:
            rospy.logwarn('dynamic config no ready yet => ignoring odom_cb')
            assert False # is this block neccesary? It seems that it does not..
            return

        # using path as a local reference avoids concurrency problems with execute_cb()
        path = self._path
        if path is None: # not tracking any path currently
            # command robot to stop
            twist = geometry_msgs.msg.Twist()
            #twist.linear.x = velocity
            #twist.angular.z = angular_vel
            self._pub.publish(twist)
            #
            return
        path_frame = self._path_frame # concurrency problems?
        reverse    = self._reverse    # concurrency problems?

        # calculate robot pose in path frame coords system
        from_frame = path_frame
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
        # robot pose (in 2D coords)
        robot_position = Point( robot_position[0], robot_position[1] )
        robot_heading = tf_conversions.transformations.euler_from_quaternion(robot_orientation)[2]
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
        self._dist2goal = dist2goal  # used to send feedback to the action server client

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
        v_max = path.velocity_at(robot_position)
        a_max = self._config['a_max']

        #
        if self._tracking_done is None: # tracking not started
            self._tracking_done = False # set tracking to in-progress
            self._velocity = 0  # current velocity

        # speed control
        # speed up
        velocity = self._velocity
        if velocity < v_max:
            velocity += a_max * dt
            velocity = min(v_max, velocity)
        # slow down
        if velocity > v_max:
            velocity -= a_max * dt
            if velocity < v_max:
                velocity = v_max
        # start breaking for goal?
        v = path_tracking.kinematics.calc_vel_from_d2g_1dof(dist2goal, a_max)
        if v <= velocity:  # start breaking for goal?
            velocity = v
        # goal reached? => latch to goal
        if dist2goal <= 0:
            velocity = 0
            self._tracking_done = True
        #
        self._velocity = velocity

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
