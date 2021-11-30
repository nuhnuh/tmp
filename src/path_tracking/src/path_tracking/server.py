#! /usr/bin/env python

import threading
from abc import abstractmethod

import rospy

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



class Server(object):


    def __init__(self):

        rospy.init_node('path_tracking')

        #
        self._odom_frame = rospy.get_param('~odom_frame', '')
        assert(self._odom_frame!=''), 'an odom_frame is required!'
        rospy.loginfo('odom_frame: %s' % self._odom_frame)

        self._lock = threading.Lock() # protects internal variables affected by concurrency

        #
        self._reset_state()

        # init action server
        self._as = actionlib.SimpleActionServer(
                rospy.get_name(),
                path_tracking.msg.TrackPathAction,
                execute_cb=self._execute_cb,
                auto_start=False )
        self._as.start()
        rospy.loginfo('path_tracking action server initialized and waiting for requests')


    def _reset_state(self):
        with self._lock:
            self._path = None
            self._tracking_done = None
            self._dist2goal = float('Infinity')


    def run(self):

        #self._tf_listener = tf.TransformListener()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        self._tfBuffer = tfBuffer

        # create publishers for dbg info
        self._pub_heading_err    = rospy.Publisher('~heading_err',    std_msgs.msg.Float32,          queue_size=1)
        self._pub_cte            = rospy.Publisher('~cte',            std_msgs.msg.Float32,          queue_size=1)

        # control loop publishes cmd_vel / ackermann_cmd
        rospy.loginfo('control loop begin')
        rate = rospy.Rate(50.0)
        last_time = rospy.get_rostime()
        while not rospy.is_shutdown():
            t = rospy.get_rostime()
            delta_t = (t - last_time).to_sec()
            last_time = t

            if delta_t == 0.: # first time
                rate.sleep()
                continue
            assert( delta_t > 0.), 'WTF???'

            with self._lock:
                self._cmd_loop_iter(t, delta_t)

            rate.sleep()


    @abstractmethod
    def _cmd_loop_iter(self, t_now, dt):
        pass


    def _execute_cb(self, goal_msg):
        rospy.loginfo('[**************************************nuhnuh DBG]: server._execute_cb()')

        # publish debug info
        rospy.loginfo('%s: path with %d points received!' % (rospy.get_name(), len(goal_msg.path_x)))

        # helper variables
        rate = rospy.Rate(1)  # feedback rate

        # tag tracking as started (None: not started, False: started, True: completed and successful)
        with self._lock:
            assert(self._tracking_done is None), 'WTF??? (%s)' % self._tracking_done

        # path tracking internal state
        # TODO: ensure vehicle stops if interrupted?

        # extract path from message
        self._path_frame = goal_msg.frame_id # this must be before setting self._path to avoid concurrency problems
        self._reverse = goal_msg.reverse
        #
        x = goal_msg.path_x
        y = goal_msg.path_y
        v = goal_msg.path_v
        path = LinearPWPath(x, y, v)
        with self._lock:
            assert(self._path == None), 'WTF???'
            self._path = path

        # publish path for rviz visualization
        _pub_path_to_rviz(path, self._path_frame)

        # start executing the action
        count = 0
        while not rospy.is_shutdown():
            rospy.loginfo('%s: path tracking step %d' % (rospy.get_name(), count))
            count += 1

            with self._lock:
                tracking_done = self._tracking_done
                dist2goal = self._dist2goal

            # check if accomplised
            if tracking_done:
                rospy.loginfo('%s: path tracking accomplished!' % rospy.get_name())
                # send result
                result = path_tracking.msg.TrackPathResult()
                result.dist2goal = dist2goal
                self._as.set_succeeded(result)
                break
            # TODO: abort if something bad happens
            #self._as.set_aborted() ?
            # check if preempt has been requested
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: path tracking preempted!' % rospy.get_name())
                self._as.set_preempted()
                break
            # send feedback
            feedback = path_tracking.msg.TrackPathFeedback()
            feedback.dist2goal = dist2goal
            self._as.publish_feedback(feedback)
            #
            rate.sleep()

        self._reset_state()
        rospy.loginfo('%s: path tracking cleanly ends!' % rospy.get_name())


def _pub_path_to_rviz(path, frame_id):
    """Publishes the path beeing tracked to rviz"""

    #rospy.loginfo('------------------- _pub_path_to_rviz begin')

    # path to poses
    import geometry_msgs.msg
    import tf_conversions
    poses = []
    for idx in range(len(path.x)):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = path.x[idx]
        pose.position.y = path.y[idx]
        heading = path.normal_at_waypoint(idx).rot270().heading()
        quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, heading)
        pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)
        poses.append( pose )

    # create path message
    import nav_msgs.msg
    path_msg = nav_msgs.msg.Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = rospy.Time.now()
    path_msg.poses = [] # geometry_msgs/PoseStamped[] poses
    #
    for pose in poses:
        pose_msg = geometry_msgs.msg.Pose()
        pose_msg.position    = pose.position
        pose_msg.orientation = pose.orientation
        pe_msg = geometry_msgs.msg.PoseStamped()
        pe_msg.pose = pose_msg
        path_msg.poses.append(pe_msg)

    # create publisher
    path_pub = rospy.Publisher('path', nav_msgs.msg.Path, queue_size=1, latch=True)

    # publish path
    path_pub.publish(path_msg)

    #rospy.loginfo('------------------- _pub_path_to_rviz end')



