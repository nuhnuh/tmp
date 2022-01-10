#!/usr/bin/env python
# TIP: based on modbus_wrapper_client.py


try:
    from pymodbus.client.sync import ModbusTcpClient
except Exception, e:
    print "pymodbus not found!\nInstall it by:\nsudo apt install python-pymodbus"
    print e
    raise e
from threading import Lock


HOST = "10.10.6.110"
PORT = 502
RATE = 50

NUM_READ_REGISTERS  = 2
NUM_WRITE_REGISTERS = 3
READ_REGISTERS_ADDRESS = 40000
WRITE_REGISTERS_START  = 40020


#class ModbusWrapperClient():
class TecnaTurtlePlcClient:

    def __init__(self, host, port=502, rate=50):
        try:
            self.client = ModbusTcpClient(host, port)
        except Exception, e:
            #rospy.logwarn()
            print "Could not get a modbus connection to %s:%d" % (host, port)
            raise e

        self.__rate = rate

        self.__mutex = Lock()

        self.__sub = rospy.Subscriber(sub_topic,HoldingRegister,self.__updateModbusOutput,queue_size=500)
        self.__pub = rospy.Publisher(pub_topic,HoldingRegister,queue_size=500, latch=True)

        rospy.on_shutdown(self.closeConnection)

    def __updateModbusInput(self,delay=0):
        """                
            Loop that is listening to the readable modbus registers and publishes it on a topic
            :param delay: The delay time until the loop starts
            :type delay: float 
        """
        rospy.sleep(delay)
        self.listener_stopped = False
        self.stop_listener = False
        update = True
        while not rospy.is_shutdown() and self.stop_listener is False:
            try: 
                if not rospy.is_shutdown() :
                    tmp =  self.readRegisters()
                    if tmp is None:
                        rospy.sleep(2)
                        continue
                    # rospy.logwarn("__updateModbusInput tmp is %s ", str(tmp))
                    # rospy.logwarn("__updateModbusInput self.__input.data is %s ", str(self.__input.data))

                    if tmp != self.__input.data:
                        update = True
                        self.__input.data = tmp
                    else:
                        update = False 
            except Exception,e:
                rospy.logwarn("Could not read holding register. %s", str(e))
                raise e
                rospy.sleep(2)
        
            if update:
                if self.__pub.get_num_connections() > 0:
                    try:
                        self.__pub.publish(self.__input)
                    except Exception,e:
                        rospy.logwarn("Could not publish message. Exception: %s",str(e))
                        raise e
            rospy.Rate(self.__rate).sleep()
        self.listener_stopped = True

    def closeConnection(self):
        """Closes the modbus connection"""
        self.client.close()



def test1():

    # define modbus connection to the PLC
    host = HOST
    port = PORT
    client = ModbusTcpClient(host, port)

    # dbg: check that server is reachable
    def checkPlcConnection():
        connected = client.connect()
        if not connected:
            #rospy.logwarn("Could not get a modbus connection to %s:%d" % (host, port))
            print "Could not get a modbus connection to %s:%d" % (host, port)
            raise "Could not get a modbus connection to %s:%d" % (host, port)
        print "ModbusTcpClient connected to %s:%d" % (host, port)
    checkPlcConnection()

    # read from PLC
    def readFromPlc():
        address = READ_REGISTERS_ADDRESS  # reading registers start address
        num_registers = NUM_READ_REGISTERS
        try:
            values = client.read_holding_registers(address, num_registers).registers
            print "plc_registers[%d..] readed: %s" % (address, str(values))
        except Exception, e:
            #rospy.logwarn("Could not read on address %d. Exception: %s", address, str(e))
            print "Could not read on address %d. Exception: %s" % (address, str(e))
            raise e
    readFromPlc()

    # write to PLC
    def writeToPlc():
        address = WRITE_REGISTERS_START  # writing registers start address
        values = [100, 101, 103]
        try:
            client.write_registers(address, values)
            print "plc_registers[%d..] written: %s" % (address, str(values))
        except Exception, e:
            #rospy.logwarn("Could not write values %s to address %d. Exception %s",str(values),address, str(e))
            print "Could not write values %s to address %d. Exception %s" % (str(values), address, str(e))
            raise e
    writeToPlc()

    # close
    client.close()


def main():
    client = TecnaTurtlePlcClient(HOST, PORT, RATE)
    try:
        client.spin()
    except rospy.ROSInterruptException:
        print "rospy.ROSInterruptException"


if __name__ == '__main__':
    #import pdb; pdb.set_trace()
    print("test1..")
    test1()
    print("..test1")
    print("--------------------------")
    print("main..")
    main()
    print("..main")
