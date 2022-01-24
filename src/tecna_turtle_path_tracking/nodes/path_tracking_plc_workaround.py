#! /usr/bin/env python

import rospy
import std_msgs.msg
import actionlib

import path_tracking.msg



def _load_path(path_fn):
    import os
    assert(os.path.exists(path_fn)), 'file not found (%s)' % path_fn

    from path_tracking.geometry2d import LinearPWPath
    path = LinearPWPath.load(path_fn)
    rospy.loginfo('path %s loaded', format(path_fn))
    return path


class PathTrackingPlcWorkaround:


    def __init__(self):

        self._load_paths_database()

        self._path_id_requested = None
        def path_id_cb(msg):
            self._path_id_requested = msg.data
        self._path_id_subs = rospy.Subscriber("path_id", std_msgs.msg.Int32, path_id_cb, queue_size=1)

        self._path_id_pub = rospy.Publisher("path_tracking_status", std_msgs.msg.Int32, queue_size=1)


    def _load_paths_database(self):
        # TODO: load paths database
        self._paths_db = {1: path_1, 2: path_2}


    def run(self):
        rate = rospy.Rate(10)
        path_id = None
        while not rospy.is_shutdown():

            if self._path_id_requested != path_id:
                self._path_id_requested = path_id
                self._processPathTrackingRequest()

            self._path_id_pub.publish(std_msgs.msg.Int32(0))

            rate.sleep()


        rospy.logwarn("run")


    def _processPathTrackingRequest(self):
        # Creates the SimpleActionClient, passing the type of the action
        # (TrackPathAction) to the constructor.
        client = actionlib.SimpleActionClient('/path_tracking', path_tracking.msg.TrackPathAction)

        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo('-----------------------waiting for server..')
        client.wait_for_server()
        rospy.loginfo('-----------------------..waiting for server')

        # Creates a goal to send to the action server
        goal = path_tracking.msg.TrackPathGoal(frame_id=frame_id, path_x=path.x, path_y=path.y, path_v=path.v, reverse=reverse)

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
                print("  status:", status)
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


def main():
    rospy.init_node('path_tracking_plc_workaround', anonymous=True)
    workaround = PathTrackingPlcWorkaround()
    workaround.run()



if __name__ == '__main__':
    main()
