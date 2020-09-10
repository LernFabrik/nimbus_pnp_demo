#!/usr/bin/env python
import rospy
from iwtros_msgs.msg import plcControl 
from iwtros_msgs.msg import kukaControl

from time import sleep
import snap7 as sn
from snap7.util import *
import struct

READ_AREA = 0x82     #area for Q memory
WRITE_AREA = 0x81    #area for I memory
START = 200
LENGTH = 1
BIT0 = 0
BIT1 = 1
BIT2 = 2

class _Control(object):
    def __init__(self, *args, **kwargs):
        super(_Control, self).__init__(*args, **kwargs)
        controllerTopic = "/iiwa/plc_control"
        listenerTopic = "/iiwa/plc_listner"
        self.pub = rospy.Publisher(controllerTopic, plcControl, queue_size=10)
        self.sub = rospy.Subscriber(listenerTopic, kukaControl, self.Callback)

        self._plc_read = sn.client.Client()
        self._plc_read.connect("192.168.0.1", 0, 1)
        self._plc_write = sn.client.Client()
        self._plc_write.connect("192.168.0.1", 0, 1)

        self._reached_home = False
        self._placed_conveyor = False
        self._placed_DHBW = False
        return
    

    def __del__(self):
        self._plc_read.destroy()
        self._plc_write.destroy()


    def Callback(self, data):
        mByte = self._plc_write.read_area(READ_AREA, 0, START, LENGTH)
        set_bool(mByte, 0, BIT0, 0)
        set_bool(mByte, 0, BIT1, 0)
        set_bool(mByte, 0, BIT2, 0)
        if(data.ReachedHome):
            set_bool(mByte, 0, BIT0, 1)
            self._plc_write.write_area(WRITE_AREA, 0, START, mByte)
            rospy.loginfo("Reached Home")
            self._reached_home = True
        if(data.ConveyorPlaced):
            set_bool(mByte, 0, BIT2, 1)
            self._plc_write.write_area(WRITE_AREA, 0, START, mByte)
            rospy.loginfo("Placed on conveyor belt")
            self._placed_conveyor =True
            #rospy.sleep(2)
        if(data.DHBWPlaced):
            set_bool(mByte, 0, BIT1, 1)
            self._plc_write.write_area(WRITE_AREA, 0, START, mByte)
            rospy.loginfo("Placed on DHBW belt")
            self._placed_DHBW = True
        self.waitForUpdate = False


    def pubControl(self, home = False, conveyor = False, DHBW = False):
        cntrMsg = plcControl()
        cntrMsg.MoveHome = home
        cntrMsg.ConveyorPickPose = conveyor
        cntrMsg.DHBWPickPose = DHBW
        self.pub.publish(cntrMsg)
    
    def _cntr_loop(self):
        rate = rospy.Rate(5)
        # Initialize the motion
        mByte = self._plc_read.read_area(READ_AREA, 0, START, LENGTH)
        wait_for_home = get_bool(mByte, 0, BIT0)
        while not wait_for_home and not rospy.is_shutdown():
            mByte = self._plc_read.read_area(READ_AREA, 0, START, LENGTH)
            wait_for_home = get_bool(mByte, 0, BIT0)
            rate.sleep()   
            
        # MoveHome
        print("Move Home")
        self.pubControl(home=True, conveyor=False, DHBW=False)
        # Wait for reached home
        while not self._reached_home and not rospy.is_shutdown():
            print("Waiting for robot to complete the motion")
            rate.sleep()
        
        # Main loop START
        while self._reached_home and not rospy.is_shutdown():
            # Wait for Conveyor belt or DHBW Picking command for plc   
            mByte = self._plc_read.read_area(READ_AREA, 0, START, LENGTH)
            wait_for_DBHW = get_bool(mByte, 0, BIT1)
            wait_for_conveyor = get_bool(mByte, 0, BIT2)
            self._placed_conveyor = False
            self._placed_DHBW = False

            if wait_for_DBHW:
                self._reached_home = False
                rospy.loginfo("Pick from DHBW and place conveyor belt")
                # Send command to KUKA iiwa 7
                self.pubControl(home=False, conveyor=False, DHBW=True)
                while not self._placed_conveyor and not rospy.is_shutdown():
                    rate.sleep()
                
                # MoveHome
                self.pubControl(home=True, conveyor=False, DHBW=False)
                # Wait for reached home
                while not self._reached_home and not rospy.is_shutdown():
                    rate.sleep()
            
            if wait_for_conveyor:
                self._reached_home = False
                rospy.loginfo("Pick from conveyor belt and place DHBW")
                # Send command to KUKA iiwa 7
                self.pubControl(home=False, conveyor=True, DHBW=False)
                while not self._placed_DHBW and not rospy.is_shutdown():
                    rate.sleep()
                
                # MoveHome
                self.pubControl(home=True, conveyor=False, DHBW=False)
                # Wait for reached home
                while not self._reached_home and not rospy.is_shutdown():
                    rate.sleep()

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("plc_control_node", anonymous=False)
    con = _Control()
    try:
        con._cntr_loop()
    except rospy.ROSInternalException:
        del con
        pass