#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import os
import threading, time
import sys

sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/doosan-robot/common/imp"%HOME_DIR)) )

# for single robot 
ROBOT_ID     = "dsr"
ROBOT_MODEL  = ""
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *



class GripperControl:
    def __init__(self):
        rospy.init_node('snu_gripper', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        
        rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1)
        self.srv_gripper_send_data = rospy.ServiceProxy(ROBOT_ID + ROBOT_MODEL + '/io/set_tool_digital_output', SetToolDigitalOutput)

        self.pub_stop = rospy.Publisher(ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
        self.init_flange()

    def init_flange(self):
        self.srv_gripper_send_data(0, 0)
        self.srv_gripper_send_data(1, 0)
        self.srv_gripper_send_data(2, 0)
        self.srv_gripper_send_data(3, 0)
        self.srv_gripper_send_data(4, 0)
        self.srv_gripper_send_data(5, 0)

    def close(self):
        self.srv_gripper_send_data(2, 1)
        self.init_flange()


    def open(self):
        self.srv_gripper_send_data(1, 1)
        self.init_flange()

    def shutdown(self):
        print "shutdown time!"
        print "shutdown time!"
        print "shutdown time!"
        
        self.pub_stop.publish(stop_mode=1) #STOP_TYPE_QUICK)
        return 0

    def pnp_cb(self, msg):
        if msg.data == "close":
            print "Gripper: close"
            self.close()
            rospy.sleep(1)
        
        if msg.data == "open":
            print "Gripper: open"
            self.open()
            rospy.sleep(1)
        if msg.data == "2.0":
            print "SNU IDIM Demo Start!!!"
            
            #p0 = posj(-0.29, 2.76, -109.19, -0.82, -71.49, 1.12)
            ##p1 = posj(-0.92, -15.61, -127.27, -1, -36.5, 1)
	        #p1 = posj(-0.84, -20.21, -126.6, -1.02, -31.81, 1.12)
            #p2 = posj(-90.04, -4.69, -94.56, -0.12, -76.05, 1.12)
            #p3 = posj(-84.23, -18.93, -99.84, -0.26, -60.67, 4.62)
            p0 = posj(3.05, 1.18, -105.17, -0.61, -75.53, 4.01)
            p1 = posj(2.24, -21.83, -124.98, -0.72, -33.04, 3.83)
            p2 = posj(-79.28, -44.42, -34.49, -0.34, -102.54, 3.65)
            p3 = posj(-79.45, -46.55, -64.83, -0.25, -71.00, 3.64)

            movej(p0, vel=60, acc=30)
            movej(p1, vel=60, acc=30)
            gripper.close()
            rospy.sleep(1)
            movej(p0, vel=60, acc=30)
            movej(p2, vel=60, acc=30)
            movej(p3, vel=60, acc=30)
            gripper.open()
            rospy.sleep(1)
            movej(p2, vel=60, acc=30)
            movej(p0, vel=60, acc=30)
        




if __name__ == "__main__":

    gripper = GripperControl()   
    

    while not rospy.is_shutdown():
        pass

        

    print 'good bye!'
