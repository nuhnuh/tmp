#!/usr/bin/env python
"""
Estimates vehicle pose, sends odom messages and broadcasts odom tf
Vehicle position relative to a starting location is estimated using the measured wheel speed

TODO: get wheel_radius and track_width from urdf if it is posible !!!!!!!!
"""


import rospy
import sys
import tf # TODO: use tf2 instead! (see /home/manu/Manu/Jobs/UPNA/JdA/VELETA/robots/tmp/kobuki_ws/src/path_tracking/src/path_tracking/server.py)
import geometry_msgs
import nav_msgs.msg
import sensor_msgs
import math
import numpy as np

from tecna_turtle_msgs.msg import DiffDriveWheelVel



class DiffDriveOdometryEstimator(object):
    """
    Reads DiffDriveWheelVel messages from the wheel_vel_odom topic, estimates the odometry pose and generates Odometry messages and tf
    """


    def __init__(self):
        """
        """

        rospy.init_node('odometry_estimator')

        # TODO: avoid namespace?
        ns = rospy.get_namespace()
        self.ns = ns

        # get params
        track_width, wheel_radius, wheel_vel_odom_topic = _get_params()
        self._track_width = track_width
        self._wheel_radius = wheel_radius
        def dbg_print_params():
            print "self._track_width:", self._track_width
            print "self._wheel_radius:", self._wheel_radius
        dbg_print_params()

        # Subscribe to /wheel_vel_odom
        topic = '{}{}'.format(ns, wheel_vel_odom_topic)
        rospy.Subscriber(topic, DiffDriveWheelVel, self._wheel_vel_odom_cb)

        # Create /odom publisher
        topic = '{}odom2'.format(ns)  # TODO: should be odom no odom2 (which is only for dbg)
        self._odom_pub = rospy.Publisher(topic, nav_msgs.msg.Odometry, queue_size=1)

        # Create odom frame tf broadcaster
        tfbr = tf.TransformBroadcaster()
        self._tfbr = tfbr

        # Initialize vehicle pose estimation
        self._last_wheel_vel_msg_time = None  # TODO: unused?
        self.position_x = None
        self.position_y = None
        self.heading    = None

        #
        rospy.spin()
        rospy.logwarn('odometry_estimator ended!')


#    def _wheel_vel_odom_cb( self, joint_state_msg ):
    def _wheel_vel_odom_cb(self, msg):
        #print( msg )

        #t = msg.header.stamp.to_sec()
        t = rospy.Time.now().to_sec()


        # first time initialization
        if self._last_wheel_vel_msg_time is None:
            self._last_wheel_vel_msg_time = t
            # initialize pose (it will track the pose of the diff_drive axle frame relative to the odom frame)
            self.position_x, self.position_y, self.heading = .0, .0, .0
            return
        delta_t = t - self._last_wheel_vel_msg_time
        assert delta_t > 0., "WTF??"
        self._last_wheel_vel_msg_time = t



#        # obtain wheel_joint_vel from linear_speed & angular_speed
#        # TIP: https://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
#        left_wheel_joint_vel  = linear_speed - angular_speed * track_width / 2 / wheel_radius
#        right_wheel_joint_vel = linear_speed + angular_speed * track_width / 2 / wheel_radius

        #
        left_wheel_joint_vel  = msg.left_wheel_joint_vel
        right_wheel_joint_vel = msg.right_wheel_joint_vel


        wheel_radius = self._wheel_radius
        track_width = self._track_width



        # estimate the new pose of the robot (base_link) in the coords frame of the odom frame
        # TIP:
        # - the kinematic ecuations track the pose of the rear_axle of the robot, which in this case is equal to the pose of the robot (the pose of its base_link)
        # - the estimation is first performed on the rear_axle coords at the previous timestep and then transformed to the odom frame
        # description of the geometry involved:
        #   R: distance from the diff_drive axle to center of curvature (ICR)
        #   w: angular velocity of the vehicle around center of curvature (ICR)
        # TIP: base_link is in the middle of rear_axle and front_wheel

        if abs(right_wheel_joint_vel - left_wheel_joint_vel) < 1e-9:
            #R = Inf
            w = (right_wheel_joint_vel - left_wheel_joint_vel) * wheel_radius / track_width

            delta_x = wheel_radius * (right_wheel_joint_vel + left_wheel_joint_vel) / 2. * delta_t
            delta_y = 0.
            delta_heading = 0.
        else:
            R = (right_wheel_joint_vel + left_wheel_joint_vel) / (right_wheel_joint_vel - left_wheel_joint_vel) / 2.
            w = (right_wheel_joint_vel - left_wheel_joint_vel) * wheel_radius / track_width

            delta_x = R * math.sin( w * delta_t )
            delta_y = np.sign(w) * R * ( 1 - math.cos( w * delta_t ) )
            delta_heading = w * delta_t

        #print("  delta_t, delta_x, delta_y, delta_h: {}, {}, {}, {}".format( delta_t, delta_x, delta_y, delta_heading ))
        # rear_axle pose delta (in odom coords)
        h = self.heading
        c, s = math.cos(h), math.sin(h)
        delta_x_oc = c * delta_x - s * delta_y
        delta_y_oc = s * delta_x + c * delta_y
        delta_heading_oc = delta_heading
        # new rear_axle pose (in odom coords)
        position_x = self.position_x + delta_x_oc
        position_y = self.position_y + delta_y_oc
        heading = self.heading + delta_heading_oc
        # update rear_axle pose (in odom coords)
        self.position_x, self.position_y, self.heading = position_x, position_y, heading
        #print("  position_x, position_y, heading: {:.2f}, {:.2f}, {:.1f}".format( position_x, position_y, math.degrees(heading) ))


        v = wheel_radius * (right_wheel_joint_vel + left_wheel_joint_vel) / 2
        #print "v: {}, w: {}".format(v, w)
        self._publish(t, v, w)


    def _publish(self, t, v, w):
        position_x, position_y, heading = self.position_x, self.position_y, self.heading

        stamp = rospy.Time.from_sec(t)  # TODO: better use the time of the msg received in the callback 1 level above

        # publish odom to base_link tf (remember that base_link frame is identical to the diff_drive axle frame)
        position = [ position_x, position_y, 0. ]
        import tf_conversions
        orientation = tf_conversions.transformations.quaternion_from_euler(0, 0, heading)
        # TIP ::sendTransform() http://docs.ros.org/jade/api/tf/html/python/tf_python.html
        transform_from = "{}odom".format(self.ns)
        transform_to   = "{}base_link".format(self.ns)
        self._tfbr.sendTransform(position, orientation, stamp, transform_to, transform_from)

        # publish odom to base_link msg (nav_msgs.msg.Odometry)
        odom = nav_msgs.msg.Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = transform_from
        odom.child_frame_id  = transform_to
        odom.pose.pose.position = geometry_msgs.msg.Point(position_x, position_y, 0.)
        quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, heading)  # TODO: already done above?
        odom.pose.pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)
        #odom.pose.covariance = # TODO
        odom.twist.twist.linear  = geometry_msgs.msg.Vector3(v, 0, 0)
        odom.twist.twist.angular = geometry_msgs.msg.Vector3(0, 0, w)
        #odom.twist.covariance = # TODO
        self._odom_pub.publish(odom)


def _get_params():

    def get_param(param_name):
        if not rospy.has_param(param_name):
            rospy.logerr("param %s not found" % param_name)
            raise ValueError()
        param_value = rospy.get_param(param_name)
        return param_value

    # track_width (distance between wheel centers
    track_width  = get_param("~track_width")
    track_width = float(track_width)
    if track_width <= 0:
        rospy.logerr("track_width = %f <= 0 !!!!")
        raise ValueError()

    # wheel_radius
    wheel_radius  = get_param("~wheel_radius")
    wheel_radius = float(wheel_radius)
    if wheel_radius <= 0:
        rospy.logerr("wheel_radius = %f <= 0 !!!!")
        raise ValueError()

    wheel_vel_odom_topic = "wheel_vel_odom"
    wheel_vel_odom_topic = "wheel_vel_cmd"  # TODO: remove this! is just for debug

    return track_width, wheel_radius, wheel_vel_odom_topic


def main():
    oe = DiffDriveOdometryEstimator()


if __name__ == '__main__':
    main()

