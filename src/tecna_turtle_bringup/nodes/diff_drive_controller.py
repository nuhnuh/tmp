#!/usr/bin/env python

"""
Transforms Twist messages into DiffDriveWheelVel messages
TIP: based in ackermann_controller.py

"""

import math  # from math import pi

import rospy
from geometry_msgs.msg import Twist

from tecna_turtle_msgs.msg import DiffDriveWheelVel


class DiffDriveCtrlr:
    """
    An object of class DiffDriveCtrlr is a node that controls the wheels of a differential wheeleld vehicle.
    Gets Twist messages from the cmd_vel topic and generates DiffDriveWheelVel messages on the cmd_wheel_vel topic.
    """


    def __init__(self):
        """Initialize this DiffDriveCtrlr"""

        rospy.init_node("DiffDriveCtrlr")
        rospy.loginfo("Initializing DiffDriveCtrlr...")

        track_width, wheel_radius, cmd_vel_topic = _get_params()
        self._track_width = track_width
        self._wheel_radius = wheel_radius

        rospy.loginfo("Subscribing to cmd_vel...")
        self._cmd_vel = rospy.Subscriber(cmd_vel_topic, Twist, self._cmd_vel_cb, queue_size=1)

        rospy.loginfo("Creating cmd_wheel_vel publisher...")
        self._wheel_vel_cmd = rospy.Publisher("/wheel_vel_cmd", DiffDriveWheelVel, queue_size=1)


    def _cmd_vel_cb(self, cmd_vel_msg):

        linear_speed  = cmd_vel_msg.linear.x
        angular_speed = cmd_vel_msg.angular.z
        wheel_radius = self._wheel_radius
        track_width = self._track_width

        # obtain wheel_joint_vel from linear_speed & angular_speed
        # TIP: https://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
        left_wheel_joint_vel  = (linear_speed - angular_speed * track_width / 2) / wheel_radius
        right_wheel_joint_vel = (linear_speed + angular_speed * track_width / 2) / wheel_radius

        # publish wheel_vel_cmd
        msg = DiffDriveWheelVel()
        msg.left_wheel_joint_vel = left_wheel_joint_vel
        msg.right_wheel_joint_vel = right_wheel_joint_vel
        self._wheel_vel_cmd.publish(msg)


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

    cmd_vel_topic = "cmd_vel"

    return track_width, wheel_radius, cmd_vel_topic


def main():
    rospy.loginfo("DiffDriveCtrlr starting..")
    DiffDriveCtrlr()
    rospy.spin()
    rospy.loginfo("..DiffDriveCtrlr ending!")


if __name__ == "__main__":
    main()

