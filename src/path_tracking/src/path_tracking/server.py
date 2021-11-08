#! /usr/bin/env python

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

        #rospy.init_node('path_tracker', anonymous=True)
        rospy.init_node('path_tracking')

        #
        self._odom_frame = rospy.get_param('~odom_frame', '')
        assert(self._odom_frame!=''), 'an odom_frame is required!'
        rospy.loginfo('odom_frame: %s' % self._odom_frame)

        #
        import threading
        self._lock = threading.Lock() # protects internal variables affected by concurrency
        self._path = None
        self._tracking_done = None

        # # dynamic_reconfigure
        # self._config = None
        # def _config_cb(config, level):
        #     print('config updated :)', level)
        #     print(config)
        #     self._config = config
        #     # recalculate breaking distance
        #     max_vel = self._config['vel']
        #     max_acel = .15
        #     _, velocity_change_distance = path_tracking.kinematics.plan_velocity_change_1dof( a_max=max_acel, v_0=max_vel, v_g=0 )
        #     print('velocity_change_distance:', velocity_change_distance)
        #     self._brake_distance = velocity_change_distance
        #     #
        #     return config
        # import dynamic_reconfigure.server
        # from path_tracking.cfg import PathTrackingConfig
        # srv = dynamic_reconfigure.server.Server(PathTrackingConfig, _config_cb)
        #
        # self._pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)


    def run(self):

        #self._tf_listener = tf.TransformListener()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        self._tfBuffer = tfBuffer

        # create publishers for dbg info
        self._pub_heading_err    = rospy.Publisher('~heading_err',    std_msgs.msg.Float32,          queue_size=1)
        self._pub_cte            = rospy.Publisher('~cte',            std_msgs.msg.Float32,          queue_size=1)

        # init action server
        self._as = actionlib.SimpleActionServer(
                rospy.get_name(),
                path_tracking.msg.TrackPathAction,
                execute_cb=self._execute_cb,
                auto_start=False )
        self._as.start()
        rospy.loginfo('Server initialized and waiting for requests')

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
        rospy.loginfo('[**************************************nuhnuh DBG]: s._execute_cb()')

        # publish debug info
        rospy.loginfo('%s: path with %d points received!' % (rospy.get_name(), len(goal_msg.path_x)))

        # helper variables
        rate = rospy.Rate(1)

        # tag tracking as started (None: not started, False: started, True: completed and successful)
        assert(self._tracking_done is None), 'WTF??? (%s)' % self._tracking_done

        # path tracking internal state
        # TODO: concurrency?
        # TODO: initialize from odom?
        # TODO: ensure vehicle stops if interrupted?


        # extract path from message
        self._path_frame = goal_msg.frame_id # this must be before setting self._path to avoid concurrency problems
        self._reverse = goal_msg.reverse
        #
        assert(self._path == None), 'WTF???'
        x = goal_msg.path_x
        y = goal_msg.path_y
        path = LinearPWPath(x,y)
        with self._lock:
            self._path = path

        # publish path for rviz visualization
        _pub_path_to_rviz(self._path, self._path_frame)

        # start executing the action
        #for i in range(1, goal_msg.order):
        #    # check that preempt has not been requested by the client
        #    if self._as.is_preempt_requested():
        #        rospy.loginfo('%s: Preempted' % self._action_name)
        #        self._as.set_preempted()
        #        success = False
        #        break
        #    # update result
        #    result.sequence.append( result.sequence[i] + result.sequence[i-1] )
        #    # publish the feedback
        #    feedback = path_tracking.msg.TrackPathFeedback()
        #    feedback.sequence = []
        #    feedback.sequence.append(result.sequence[-2])
        #    feedback.sequence.append(result.sequence[-1])
        #    self._as.publish_feedback(feedback)
        #    # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #    rate.sleep()
        count = 0
        while not rospy.is_shutdown():
            rospy.loginfo('%s: path tracking step %d' % (rospy.get_name(), count))
            count += 1
            # check if accomplised
            if self._tracking_done:
                rospy.loginfo('%s: path tracking accomplished!' % rospy.get_name())
                # send result
                result = path_tracking.msg.TrackPathResult()
                result.dist2goal = -666
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
            feedback.dist2goal = count
            self._as.publish_feedback(feedback)
            #
            rate.sleep()

        with self._lock:
            self._path = None
            self._tracking_done = None
        rospy.loginfo('%s: path tracking cleanly ends!' % rospy.get_name())


#    #@abstractmethod
#    def _odom_cb(self, data):
#
#        # using config as a local reference avoids concurrency problems caused by the config_cb
#        config = self._config
#        if not config:
#            rospy.logwarn('[nuhnuh DBG]: dynamic config no ready yet => ignoring odom_cb')
#            return
#
#        # using path as a local reference avoids concurrency problems caused by execute_cb
#        path = self._path
#        if path is None: # not tracking any path currently
#            return
#        path_frame = self._path_frame # concurrency problems?
#        reverse    = self._reverse    # concurrency problems?
#
##        # position
##        pose = data.pose.pose
##        position = Point(pose.position.x, pose.position.y)
##        # heading
##        q = pose.orientation
##        q = ( q.x, q.y, q.z, q.w )
##        import tf_conversions
##        heading_oc = tf_conversions.transformations.euler_from_quaternion(q)
###        print('heading (odom coords):', heading_oc)
##        heading_oc = heading_oc[2]
###        print('heading (odom coords):', math.degrees(heading_oc))
##
##        # _mc => map coords (without indication => odom_coords)
##        try:
##            # this is from odom to target frame
##            from_frame  = 'odom'
##            to_frame    = self._path_frame
##            transform, rotation = self._tf_listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
##        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as unused_e:
##            rospy.logwarn('could not get transfrom from %s to %s'%(from_frame, to_frame))
##            # TODO: take security actions
##            return
##        #print('---- dbg: data =', data)
##        #print('---- dbg: transform =', transform)
#        #print('---- dbg: pose =', pose)
#        to_frame = path_frame
#        from_frame = data.header.frame_id # = odom since data if from odom_cb
#        if from_frame[0] == '/':
#            from_frame = from_frame[1:]
#        else:
#            rospy.logwarn('entra xxakau4x39 (%s, %s)' % (from_frame, to_frame))
#        _pose = geometry_msgs.msg.PoseStamped()
#        _pose.header = data.header
#        _pose.pose = data.pose.pose
#        try:
#            pose_mc = self._tf_listener.transformPose(to_frame, _pose) # http://docs.ros.org/jade/api/tf/html/python/tf_python.html
#        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as unused_e:
#        except tf.LookupException as unused_e:
#            rospy.logwarn('could not transfrom pose from %s frame to %s frame (LE)' % (from_frame, to_frame))
#            return
#        except tf.ConnectivityException as unused_e:
#            rospy.logwarn('could not transfrom pose from %s frame to %s frame (CE)' % (from_frame, to_frame))
#            return
#        except tf.ExtrapolationException as unused_e:
#            rospy.logwarn('could not transfrom pose from %s frame to %s frame (EE)' % (from_frame, to_frame))
#            # TODO: take security actions
#            return
#
#        #print('---- dbg: pose_mc =', pose_mc)
#        #import sys
#        #sys.exit(1)
#        #return
#        #position_mc = 
##        cte, heading_path_mc = path.error(pose_mc.pose.position) # TODO: unused!!!!!!!!!!!
#
#        q = pose_mc.pose.orientation
#        q = ( q.x, q.y, q.z, q.w )
#        import tf_conversions
#        heading = tf_conversions.transformations.euler_from_quaternion(q)[2]
#
#        # pure-pursuit
##        # find nearest point
##        dx = pose_mc.pose.position.x - path.x0
##        dy = pose_mc.pose.position.y - path.y0
##        dirt = Vector(dx,dy).heading()
##        nearest_point_x = path.x0 + path.R * math.cos(dirt)
##        nearest_point_y = path.y0 + path.R * math.sin(dirt)
##        nearest_point = Point(nearest_point_x, nearest_point_y)
##        # pure-pursuit point is d meters ahead the path's nearest point in the path direction
##        d = self._config['pursuit_distance']
##        a = math.pi/2
##        # pursuit point after dt
##        dt = 1./100
##        a2 = self._config['vel'] * dt / ( path.R * 2 * math.pi ) * ( 2 * math.pi )
##        print('a2:',a2, 'cte:', cte)
##        a = a + a2
##        #
##        a = a if path.ccw else -a
##        x = nearest_point.x + d * math.cos(dirt+a)
##        y = nearest_point.y + d * math.sin(dirt+a)
##        pursuit_point = Point(x, y)
#        #
#        point = Point( pose_mc.pose.position.x, pose_mc.pose.position.y )
#        ppoint, heading_vec = path.project_point(point)
#        heading_vec = heading_vec.normalize()
#        d = self._config['pursuit_distance']
#        pursuit_point = Point( ppoint.x + d * heading_vec.x, ppoint.y + d * heading_vec.y )
#        #
#        dist2goal = path.distance_to_goal(ppoint)
#
#
#        # dbg: publish pursuit axis
#        # TODO: use a rviz marker instead of tf
#        import tf_conversions
#        br = tf.TransformBroadcaster()
#        t = geometry_msgs.msg.TransformStamped()
#        t.header.stamp = rospy.Time.now()
#        t.header.frame_id = to_frame
#        t.child_frame_id  = 'pursuit'
#        #
#        t.transform.translation.x = pursuit_point.x
#        t.transform.translation.y = pursuit_point.y
#        t.transform.translation.z = 0
#        q = tf_conversions.transformations.quaternion_from_euler(0, 0, heading_vec.heading())
#        t.transform.rotation.x = q[0]
#        t.transform.rotation.y = q[1]
#        t.transform.rotation.z = q[2]
#        t.transform.rotation.w = q[3]
#        br.sendTransformMessage(t)
#        # rviz marker
#        marker = visualization_msgs.msg.Marker()
#        marker.id = 0
#        marker.header.stamp    = data.header.stamp
#        marker.header.frame_id = to_frame
#        marker.type = visualization_msgs.msg.Marker.SPHERE
#        marker.ns = "pursuit"
#        marker.action = visualization_msgs.msg.Marker.ADD
#        marker.lifetime = rospy.Duration.from_sec(1)
#        marker.pose.position.x = pursuit_point.x
#        marker.pose.position.y = pursuit_point.y
#        marker.pose.orientation.x = q[0]
#        marker.pose.orientation.y = q[1]
#        marker.pose.orientation.z = q[2]
#        marker.pose.orientation.w = q[3]
#        marker.scale.x = .1
#        marker.scale.y = .1
#        marker.scale.z = .1
#        marker.color.r = 0.0
#        marker.color.g = 1.0
#        marker.color.b = 1.0
#        marker.color.a = 0.5
#        self._pub_pursuit_marker.publish(marker)
#
#
#        # heading error
#        position_mc = pose_mc.pose.position
#        pursuit_vector_lc = Vector( pursuit_point.x-position_mc.x, pursuit_point.y-position_mc.y )
#        heading_ref = pursuit_vector_lc.heading()
#        if reverse:
#            heading = heading + math.pi
#        heading_err = heading - heading_ref
#        while heading_err < -math.pi:
#             heading_err = heading_err + 2 * math.pi
#        while heading_err > +math.pi:
#             heading_err = heading_err - 2 * math.pi
##        print('heading_err:', math.degrees(heading_err))
#        # dbg:
#        self._pub_heading_err.publish(std_msgs.msg.Float32(heading_err))
##        self._pub_cte.publish(std_msgs.msg.Float32(cte))
#
#
#        # velocity control
#        #velocity_odom = data.twist.twist.linear.x
#        max_vel = self._config['vel']
#        max_acel = .15 # TODO add this to config? state?
#
#        # try to achive target velocity (no jerk control)
#        # dt
#        # TODO: dt is very noisy. 
#        # TODO: uSE AN INDEPENDENT LOOP FOR CONTROL??
#        if hasattr(self, '_t_last_odom'):
#            dt = data.header.stamp.to_sec() - self._t_last_odom.to_sec()
#            self._t_last_odom = data.header.stamp
#        else:
#            self._t_last_odom = data.header.stamp
#            return
#        #velocity = max_vel (wild start)
#        velocity = min( self._velocity + max_acel*dt, max_vel ) # slow start
#
#        ## brake control (distance based)
#        # arriving to goal?
#        #if dist2goal < .5:
#        #    velocity = velocity * (dist2goal/.5) + 0.05 * (1. - dist2goal/.5)
#
#        # brake control (constant acceleration)
#        # arriving to goal?
#        if dist2goal <= self._brake_distance:
#            #print('braking', dist2goal)
#            # bake when arriving to the goal
#            velocity_braking = path_tracking.kinematics.calc_vel_from_d2g_1dof( dist2goal, max_acel )
#            velocity = min( velocity_braking, velocity )
#
#        # ??????????? TODO: improve this! check if goal is behind and full stop
#        if dist2goal <= 0 or self._tracking_done is True: # latch to goal
#            velocity = 0
#            with self._lock:
#                if self._tracking_done is False:
#                    self._tracking_done = True
#        #
#        self._velocity = velocity
#
#
#        # publish twist
#        twist = geometry_msgs.msg.Twist()
#        twist.linear.x = velocity
#        twist.angular.z = - self._config['PID_k'] * heading_err
#        twist.angular.z = math.copysign( min(abs(twist.angular.z), self._config['max_ang_vel']) , twist.angular.z)
#        if reverse:
#            twist.linear.x  = - twist.linear.x
#        self._pub.publish(twist)


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



