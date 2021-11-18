#!/usr/bin/env python
# TIP: based on modbus_wrapper_client.py
# TODO: it is necessary to define negative numbers in the plc


import math
import numpy as np
from threading import Lock

import rospy
import tf # TODO: use tf2 instead! (see /home/manu/Manu/Jobs/UPNA/JdA/VELETA/robots/tmp/kobuki_ws/src/path_tracking/src/path_tracking/server.py)
import geometry_msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pymodbus.client.sync import ModbusTcpClient  # TIP: apt install python-pymodbus


MODBUS_DEFAULT_PORT = 502

#READ_REGISTERS_ADDRESS = 40000
READ_REGISTERS_ADDRESS = 0
#WRITE_REGISTERS_START  = 40020
WRITE_REGISTERS_START  = 0


#import signal
#def handler(signum, frame):
#    print '[DBG] handler'
#    exit(1)
#signal.signal(signal.SIGINT, handler)


class Plc:


    def __init__(self, ip, port):
         self._client = ModbusTcpClient(ip, port)


    def read(self):
        rospy.logdebug("PLC reading ..")

        address = READ_REGISTERS_ADDRESS  # reading registers start address
        num_registers = 3
        try:
            values = self._client.read_holding_registers(address, num_registers).registers
            #print "[DBG] plc_registers[%d..] readed: %s" % (address, str(values))
        except Exception, e:
            rospy.logfatal("Could not read on PLC address %d. Exception: %s", address, str(e))
            raise e
        rospy.logdebug("PLC readed values (raw): %s", values)

        assert len(values) == num_registers, "unexpected number of registers"

        # asign plc registers to variables
        w_l, w_r, trajectory_id = values

        # transform from PLC units
        w_l = values[0] / 100.  # left wheel angular velocity (rad/s)
        w_r = values[1] / 100.  # right wheel angular velocity (rad/s)
        trajectory_id = values[2]  # commanded trajectory identifier
        rospy.logdebug("PLC readed values (decoded): w_l=%f, w_r=%f, trajectory_id=%d", w_l, w_r, trajectory_id)

        return w_l, w_r, trajectory_id


    def write(self, w_l, w_r, pose, status):
        rospy.logdebug("PLC writting ..")
        rospy.logdebug("PLC values to write (decoded): w_l=%f, w_r=%f", w_l, w_r)

        # transform data to PLC units
        w_l = int(round(100 * w_l))
        w_r = int(round(100 * w_r))
        # TODO: transform pose
        x, y, heading = 10, 11, 12  # robot pose in map coords (TODO)

        # asign plc registers from variables
        values = [w_l, w_r, x, y, heading, status, self._heartbeat()]
        rospy.logdebug("PLC values to write (raw): %s", values)
        for v in values:
            assert isinstance(v, int) and 0 <= v and v <= 065536, "%s is not an uint16" % str(v)

        address = WRITE_REGISTERS_START  # writing registers start address
        try:
            self._client.write_registers(address, values)
        except Exception, e:
            rospy.logfatal("Could not write on PLC address %d. Exception: %s", address, str(e))
            raise e


    def close(self):
        self._client.close()


    def _heartbeat(self):
        if hasattr(self, '_heartbeat_last'):
            self._heartbeat_last = (self._heartbeat_last + 1) % 2
        else:
            self._heartbeat_last = 0
        return self._heartbeat_last



class DiffDriveOdometryEstimator:


    def __init__(self, track_width, wheel_radius):
        # vehicle params
        self._wheel_radius = wheel_radius
        self._track_width = track_width

        # pose estimation params
        self.x = 0.
        self.y = 0.
        self.heading = 0.


    def integrate(self, delta_t, v_l, v_r):
        """
        delta_t: time interval to integrate (s)
        v_l: velocity of the left wheel (m/s)
        v_r: velocity of the right wheel (m/s)
        """

        assert delta_t > 0., "WTF?? (delta_t <= 0)"

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

        # estimate new pose (delta_x, delta_y, delta_heading) in old pose coords
        if abs(v_r - v_l) < 1e-9: # not turning
            #R = Inf
            w = 0.

            delta_x = (v_r + v_l) / 2. * delta_t
            delta_y = 0.
            delta_heading = 0.
        else:
            R = (v_r + v_l) / (v_r - v_l) / 2.
            w = (v_r - v_l) / track_width

            delta_x = R * math.sin( w * delta_t )
            delta_y = np.sign(w) * R * ( 1 - math.cos( w * delta_t ) )
            delta_heading = w * delta_t

        # transform the new pose estimate into the odom frame coords (oc)
        h = self.heading
        c, s = math.cos(h), math.sin(h)
        #
        delta_x_oc = c * delta_x - s * delta_y
        delta_y_oc = s * delta_x + c * delta_y
        delta_heading_oc = delta_heading
        #
        x = self.x + delta_x_oc
        y = self.y + delta_y_oc
        heading = self.heading + delta_heading_oc

        # update pose
        self.x, self.y, self.heading = x, y, heading

        pose = x, y, heading
        return pose, w



class TecnaTurtlePlcClient:


    def __init__(self, rate=20):

        rospy.logdebug("TecnaTurtlePlcClient reading params ..")
        modbus_params, vehicle_params = _get_params()
        plc_ip, plc_port = modbus_params
        track_width, wheel_radius = vehicle_params
        self._track_width = track_width
        self._wheel_radius = wheel_radius

        rospy.logdebug("TecnaTurtlePlcClient initializing modbus connection ..")
        self._plc = Plc(plc_ip, plc_port)

        self._mutex = Lock()

        self._odometry_estimator = DiffDriveOdometryEstimator(track_width, wheel_radius)

        rospy.logdebug("TecnaTurtlePlcClient subscribing to cmd_vel ..")
        cmd_vel_topic = "cmd_vel"
        with self._mutex:
            self._w_l_cmd = 0
            self._w_r_cmd = 0
        self._cmd_vel_subs = rospy.Subscriber(cmd_vel_topic, Twist, self._cmd_vel_cb, queue_size=1)

        rospy.logdebug("TecnaTurtlePlcClient creating odom topic and tf publishers ..")
        # Create odom topic publisher
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        # Create odom frame tf broadcaster
        tfbr = tf.TransformBroadcaster()
        self._tfbr = tfbr

        rospy.logdebug("TecnaTurtlePlcClient entering plc I/O loop ..")
        self._rate = rospy.Rate(rate)
        try:
            self._spin()
        except Exception, e:
            rospy.logfatal("Exception in the I/O loop: %s", str(e))
            raise e

        rospy.logdebug("TecnaTurtlePlcClient closing modbus connection ..")
        #rospy.on_shutdown(self.closeConnection)
        self._plc.close()  # closes modbus connection

        rospy.logwarn("TecnaTurtlePlcClient ended")


    def _cmd_vel_cb(self, msg):

        linear_speed  = msg.linear.x  # of the center of the diff_drive axle
        angular_speed = msg.angular.z  # around the center of curvature (ICR)

        wheel_radius = self._wheel_radius
        track_width  = self._track_width

        # diff_drive kinematics
        delta_speed = angular_speed * track_width / 2
        left_wheel_angular_vel  = (linear_speed - delta_speed) / wheel_radius
        right_wheel_angular_vel = (linear_speed + delta_speed) / wheel_radius

        # cache commanded info for the PLC
        with self._mutex:
            self._w_l_cmd = left_wheel_angular_vel
            self._w_r_cmd = right_wheel_angular_vel


    def _spin(self):
        # TODO: sometimes (spetially while using log_level=rospy.DEBUG) CTRL+C leads to deadlock :/
        t = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            # loop timing control
            t_last = t
            self._rate.sleep()
            t = rospy.Time.now().to_sec()
            delta_t = t - t_last
            rospy.logdebug("delta_t: %f seconds", delta_t)

            rospy.logdebug("llega 1")
            def read_from_plc():
                w_l, w_r, trajectory_id = self._plc.read()

                def dbg_plc_io():
                    t2 = rospy.Time.now().to_sec()
                    #rospy.logdebug("plc I/O elapsed: %f seconds", t2 - t)
                    rospy.logdebug_once("plc I/O elapsed: %f seconds", t2 - t)
                dbg_plc_io()

                return w_l, w_r, trajectory_id
            data = read_from_plc()
            w_l, w_r, trajectory_id = data

            rospy.logdebug("llega 2")
            def update_and_publish_odom():
                v_l = w_l * self._wheel_radius
                v_r = w_r * self._wheel_radius
                _, w = self._odometry_estimator.integrate(delta_t, v_l, v_r)

                v = (v_l + v_r) / 2.
                self._publish_odom(t, v, w)
            update_and_publish_odom()

            rospy.logdebug("llega 3")
            def write_to_plc():
                #w_l = +1
                #w_r = +1
                with self._mutex:
                    w_l = self._w_l_cmd
                    w_r = self._w_r_cmd
                pose = None  # TODO
                status = 0  # TODO
                self._plc.write(w_l, w_r, pose, status)
            write_to_plc()

        rospy.logwarn("spin end")


    def _publish_odom(self, t, v, w):
        position_x = self._odometry_estimator.x
        position_y = self._odometry_estimator.y
        heading    = self._odometry_estimator.heading

        stamp = rospy.Time.from_sec(t)

        # publish odom to base_link tf (remember that base_link frame is identical to the diff_drive axle frame)
        position = [ position_x, position_y, 0. ]
        import tf_conversions
        orientation = tf_conversions.transformations.quaternion_from_euler(0, 0, heading)
        # TIP ::sendTransform() http://docs.ros.org/jade/api/tf/html/python/tf_python.html
        transform_from = "odom"
        transform_to   = "base_link"
        self._tfbr.sendTransform(position, orientation, stamp, transform_to, transform_from)

        # publish odom to base_link msg (nav_msgs.msg.Odometry)
        odom = Odometry()
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

    def get_required_param(param_name):
        if not rospy.has_param(param_name):
            rospy.logerr("param %s not found" % param_name)
            raise ValueError()
        param_value = rospy.get_param(param_name)
        return param_value

    def get_optional_param(param_name, default_value):
        if not rospy.has_param(param_name):
            rospy.logwarn("param %s not found, using defaults (%s)" % (param_name, default_value))
        param_value = rospy.get_param(param_name, default_value)
        return param_value

    def get_modbus_params():
        ip = get_required_param("~ip")
        port = get_optional_param("~port", MODBUS_DEFAULT_PORT)
        port = int(port)
        return ip, port
    modbus_params = get_modbus_params()

    def get_vehicle_params():
        # track_width (distance between wheel centers
        track_width  = get_required_param("~track_width")
        track_width = float(track_width)
        if track_width <= 0:
            rospy.logerr("track_width = %f <= 0 !!!!")
            raise ValueError()

        # wheel_radius
        wheel_radius  = get_required_param("~wheel_radius")
        wheel_radius = float(wheel_radius)
        if wheel_radius <= 0:
            rospy.logerr("wheel_radius = %f <= 0 !!!!")
            raise ValueError()

        return track_width, wheel_radius
    vehicle_params = get_vehicle_params()

    params = modbus_params, vehicle_params
    return params


def dbg_pymodbus():
    #import pdb; pdb.set_trace()

    # define modbus connection to the PLC
    #plc_ip = "10.10.6.110"
    plc_ip = "127.0.0.1"
    #plc_port = MODBUS_DEFAULT_PORT
    plc_port = 5020
    client = ModbusTcpClient(plc_ip, plc_port)  # TIP: does not attempt to connect

    # dbg: check that server is reachable
    def checkPlcConnection():
        print "ModbusTcpClient connecting to %s:%d .." % (plc_ip, plc_port)
        connected = client.connect()
        if not connected:
            #rospy.logwarn("Could not get a modbus connection to %s:%d" % (plc_host, plc_port))
            print "Could not get a modbus connection to %s:%d" % (plc_host, plc_port)
            raise "Could not get a modbus connection to %s:%d" % (plc_host, plc_port)
        print "  connected"
    checkPlcConnection()

    import time; time.sleep(2)

    # read from PLC
    def readFromPlc():
        print "Reading plc_registers.."
        address = READ_REGISTERS_ADDRESS  # reading registers start address
        num_registers = 6
        try:
            values = client.read_holding_registers(address, num_registers).registers
            print "  plc_registers[%d..]: %s" % (address, str(values))
        except Exception, e:
            #rospy.logwarn("Could not read on address %d. Exception: %s", address, str(e))
            print "Could not read on address %d. Exception: %s" % (address, str(e))
            raise e
    readFromPlc()

    import time; time.sleep(2)

    # write to PLC
    def writeToPlc():
        print "Writting plc_registers.."
        address = WRITE_REGISTERS_START  # writing registers start address
        values = [101, 102, 103, 104]
        try:
            client.write_registers(address, values)
            print "  plc_registers[%d..]: %s" % (address, str(values))
        except Exception, e:
            #rospy.logwarn("Could not write values %s to address %d. Exception %s",str(values),address, str(e))
            print "Could not write values %s to address %d. Exception %s" % (str(values), address, str(e))
            raise e
    writeToPlc()

    import time; time.sleep(2)

    # test IO rate
    def estimateIoRate():
        print "Estimating modbus I/O rate.."
        address = WRITE_REGISTERS_START  # writing registers start address
        values = [0, 1, 2]
        num_registers = len(values)
        num_io_operations = 1000
        import time
        t0 = time.time()
        for _ in xrange(num_io_operations):
            values = [ v + 1 for v in values]
            client.write_registers(address, values)
            values2 = client.read_holding_registers(address, num_registers).registers
            assert values == values2, "WTF?? (unexpected number of registers readed)"
        t1 = time.time()
        elapsed = t1 - t0
        print "  %d I/O operations in %f seconds" % (num_io_operations, elapsed)
    estimateIoRate()

    # close
    client.close()


def dbg_plc_class():
    ip = "127.0.0.1"
    port = 5020
    plc = Plc(ip, port)
    print "plc.read() .."
    data = plc.read()
    print "data:", data
    print "plc.write(w_l=.5, w_r=.6, pose=(1,2,3), status=666) .."
    plc.write(w_l=.5, w_r=.6, pose=(1,2,3), status=666)


def main():
    client = TecnaTurtlePlcClient()


if __name__ == '__main__':
    #rospy.init_node('tecna_turtle_plc_client')
    rospy.init_node('tecna_turtle_plc_client', log_level=rospy.DEBUG)

#    dbg_pymodbus()
#    dbg_plc_class()
    main()
