#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from numpy import clip
import math


control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09'}

key_bindings = {
    '\x41' : ( 0.1 , 0.0),
    '\x42' : (-0.1 , 0.0),
    '\x43' : ( 0.0 ,-0.1),
    '\x44' : ( 0.0 , 0.1),
    '\x20' : ( 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0)}


class DiffDriveKeyop:

    def __init__(self):
        self.max_speed = float(rospy.get_param('~max_speed',0.5))
        self.max_angular_speed = float(rospy.get_param('~max_angular_speed',0.5))
        self.cmd_topic = rospy.get_param('~topic','/cmd_vel')
        self.frame = rospy.get_param('~frame','base_link')

        self.speed_range = [-float(self.max_speed), float(self.max_speed)]
        self.angular_speed_range = [-float(self.max_angular_speed),
                                     float(self.max_angular_speed)]
        for key in key_bindings:
            key_bindings[key] = \
                    (key_bindings[key][0] * float(self.max_speed),
                     key_bindings[key][1] * float(self.max_angular_speed))

        self.speed = 0
        self.angular_speed = 0
        self.motors_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/10.0), self.pub_callback, oneshot=False)
        self.print_state()
        self.key_loop()

    def pub_callback(self, event):
        twist_msg = Twist()
        twist_msg.linear.x = self.speed
        twist_msg.angular.z = self.angular_speed
        self.motors_pub.publish(twist_msg)

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse arrows to change linear speed and angular angle')
        rospy.loginfo('\x1b[1M\rUse space to brake and tab to align wheels')
        rospy.loginfo('\x1b[1M\rPress <ctrl-c> or <q> to exit')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteer Angle: \033[32;1m%0.2f degrees/s\033[0m',
                      self.speed, math.degrees(self.angular_speed))

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def key_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while 1:
            key = self.get_key()
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.speed = 0.0
                elif key == control_keys['tab']:
                    self.angular_speed = 0.0
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.angular_speed = self.angular_speed + key_bindings[key][1]
                    self.speed = clip(
                        self.speed,
                        self.speed_range[0],
                        self.speed_range[1])
                    self.angular_speed = clip(
                        self.angular_speed,
                        self.angular_speed_range[0],
                        self.angular_speed_range[1])
                self.print_state()
            elif key == '\x03' or key == '\x71':  # ctr-c or q
                break
            else:
                continue
        self.finalize()

    def finalize(self):
        rospy.loginfo('Exiting teleop and halting vehicle...')
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.motors_pub.publish(twist_msg)
        sys.exit()


def main():
    rospy.init_node('diff_drive_keyop_node')
    keyop = DiffDriveKeyop()


if __name__ == '__main__':
    main()
