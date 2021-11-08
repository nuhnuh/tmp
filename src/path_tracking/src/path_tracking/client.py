#! /usr/bin/env python


from __future__ import print_function

import rospy
import actionlib
import path_tracking.msg



def _load_path(path_fn):
    import os
    assert(os.path.exists(path_fn)), 'file not found (%s)' % path_fn

    from path_tracking.geometry2d import LinearPWPath
    path = LinearPWPath.load(path_fn)
    rospy.loginfo('path %s loaded', format(path_fn))
    return path


class Client:

    def __init__(self):

        rospy.init_node('path_tracking_client', anonymous=True)

        frame_id = rospy.get_param('~frame_id', 'odom')
        reverse = rospy.get_param('~reverse', False)
        path_fn  = rospy.get_param('~path_fn', '')
        #
        rospy.loginfo('-----------------------frame_id: %s' % frame_id)
        rospy.loginfo('-----------------------reverse: %s' % reverse)
        rospy.loginfo('-----------------------path_fn: %s' % path_fn)

        # load the path (it is better beffor wait_for_server() in order to fail early if necessary)
        path = _load_path(path_fn)

        # Creates the SimpleActionClient, passing the type of the action
        # (TrackPathAction) to the constructor.
        client = actionlib.SimpleActionClient('/path_tracking', path_tracking.msg.TrackPathAction)

        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo('-----------------------waiting for server..')
        client.wait_for_server()
        rospy.loginfo('-----------------------..waiting for server')

        # Creates a goal to send to the action server
        goal = path_tracking.msg.TrackPathGoal(frame_id=frame_id, path_x=path.x, path_y=path.y, reverse=reverse)

        # Sends the goal to the action server.
        def done_cb(status, result):
            print("done_cb")
            from actionlib_msgs.msg import GoalStatus
            print("  status:", status)
            print("    '%d => preempted" % GoalStatus.PREEMPTED)
            print("    '%d => succeeded" % GoalStatus.SUCCEEDED)
            print("    (See: http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html)")
            if status == GoalStatus.SUCCEEDED:
                print("  result:", result)
        def feedback_cb(feedback):
            print("feedback_cb", feedback)
        def active_cb():
            print("active_cb")
        rospy.loginfo('-----------------------sending goal..')
        client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
        rospy.loginfo('-----------------------..sending goal')
        rospy.on_shutdown(client.cancel_goal)

        # Waits for the server to finish performing the action.
        rospy.loginfo('-----------------------waiting for result..')
        timedout = not client.wait_for_result( rospy.rostime.Duration(60) )
        if timedout:
            print('WARNING! wait_for_result timeout')

        # Prints out the result of executing the action
        result = client.get_result()
        if result:
            print("Result:", result.dist2goal)

