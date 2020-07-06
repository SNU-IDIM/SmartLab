#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys, os, threading, time

sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/doosan-robot/common/imp"%HOME_DIR)) )
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_IDIM_ASMR/common/imp"%HOME_DIR)) )

# for single robot 
ROBOT_ID     = "dsr"
ROBOT_MODEL  = ""
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

from IDIM_framework import *


class dsrDigitalControl:
    def __init__(self):
        rospy.init_node('snu_dsr_digital_io_controller', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        
        rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1)
        self.srv_tool_send_data = rospy.ServiceProxy(ROBOT_ID + ROBOT_MODEL + '/io/set_tool_digital_output', SetToolDigitalOutput)
        self.srv_ctrl_send_data = rospy.ServiceProxy(ROBOT_ID + ROBOT_MODEL + '/io/set_digital_output'     , SetCtrlBoxDigitalOutput)

        self.pub_stop = rospy.Publisher(ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
        self.init_tool_digital_output()
        self.init_ctrl_digital_output()
        self.srv_analog_read = rospy.ServiceProxy(ROBOT_ID + ROBOT_MODEL + '/io/get_analog_input'     , GetCtrlBoxAnalogInput)


    def init_tool_digital_output(self):
        self.srv_tool_send_data(1, 0)
        self.srv_tool_send_data(2, 0)
        self.srv_tool_send_data(3, 0)
        self.srv_tool_send_data(4, 0)
        self.srv_tool_send_data(5, 0)
        self.srv_tool_send_data(6, 0)

    def gripper_close(self):
        #reset digital io
        self.srv_ctrl_send_data(13,0)
        self.srv_ctrl_send_data(14,0)
        rospy.sleep(1)
        
        #send closing command
        self.srv_ctrl_send_data(13,1)

    def gripper_open(self):
        self.srv_ctrl_send_data(13,0)
        self.srv_ctrl_send_data(14,0)
        rospy.sleep(1)
        #send closing command
        self.srv_ctrl_send_data(14,1)

    def gripper_signal_read(self):
        gripper_signal = self.srv_analog_read(1)
    '''
      DSR CtrlBox Digital Control
    '''
    def init_ctrl_digital_output(self):
        self.srv_ctrl_send_data( 1, 0)
        self.srv_ctrl_send_data( 2, 0)
        self.srv_ctrl_send_data( 3, 0)
        self.srv_ctrl_send_data( 4, 0)
        self.srv_ctrl_send_data( 5, 0)
        self.srv_ctrl_send_data( 6, 0)
        self.srv_ctrl_send_data( 7, 0)
        self.srv_ctrl_send_data( 8, 0)
        self.srv_ctrl_send_data( 9, 0)
        self.srv_ctrl_send_data(10, 0)
        self.srv_ctrl_send_data(11, 0)
        self.srv_ctrl_send_data(12, 0)
        self.srv_ctrl_send_data(13, 0)
        self.srv_ctrl_send_data(14, 0)
        self.srv_ctrl_send_data(15, 0)
        self.srv_ctrl_send_data(16, 0)
    
    def toolchanger_detach(self):
        self.srv_ctrl_send_data(2, 1)

    def toolchanger_attach(self):
        self.srv_ctrl_send_data(2, 0)

    def compressor_on(self):
        self.srv_ctrl_send_data(1, 1)

    def compressor_off(self):
        self.srv_ctrl_send_data(1, 0)


    def shutdown(self):
        print "shutdown time!"
        print "shutdown time!"
        print "shutdown time!"
        
        self.pub_stop.publish(stop_mode=1) #STOP_TYPE_QUICK)
        return 0

    def pnp_cb(self, msg):
        if msg.data == ACTION_IO_GRIPPER_CLOSE:
            print "Gripper: close"
            self.gripper_close()
            rospy.sleep(1)
        
        if msg.data == ACTION_IO_GRIPPER_OPEN:
            print "Gripper: open"
            self.gripper_open()
            rospy.sleep(1)

        if msg.data == ACTION_IO_TOOLCHANGER_DETACH:
            print "Tool Changer: Detach"
            self.toolchanger_detach()
            rospy.sleep(1)

        if msg.data == ACTION_IO_TOOLCHANGER_ATTACH:
            print "Tool Changer: Attach"
            self.toolchanger_attach()
            rospy.sleep(1)

        if msg.data == ACTION_IO_COMPRESSOR_ON:
            print "Compressor: ON"
            self.compressor_on()
            rospy.sleep(1)

        if msg.data == ACTION_IO_COMPRESSOR_OFF:
            print "Compressor: OFF"
            self.compressor_off()
            rospy.sleep(1)

if __name__ == "__main__":

    snu_dsr_io_ctrl = dsrDigitalControl()   
    

    while not rospy.is_shutdown():
        pass

        

    print 'good bye!'
