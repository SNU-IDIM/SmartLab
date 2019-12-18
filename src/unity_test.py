#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
import sys
from math import pi
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray


sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/doosan-robot/common/imp"%HOME_DIR)) )
NS_           = "R_001"
ROBOT_ID_     = "dsr"
ROBOT_MODEL_  = ""
import DR_init
DR_init.__dsr__id = NS_+'/'+ROBOT_ID_
DR_init.__dsr__model = ROBOT_MODEL_
from DSR_ROBOT import *

EPSILON = 0.0000001
DEG2RAD = pi / 180.0
RAD2DEG = 180.0 / pi
M2MM = 1000.0
MM2M = 1.0/1000.0

test = JointState()



class SNUDSR:
    def __init__(self):
        rospy.init_node('dsr_simple_test_py')
        rospy.on_shutdown(self.shutdown)
        self.target_joint_states = Float64MultiArray()
        
        self.count = 0

        self.pub_stop = rospy.Publisher(ROBOT_ID_ +ROBOT_MODEL_ +'/stop', RobotStop, queue_size=10)
        self.pub_unity = rospy.Publisher('/R_001/dsr/dsr_joint_position_controller/command', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/unity/joint_states', JointState, self.unity_cb, queue_size=1)
        #rospy.Subscriber('/R_001/dsr_cmd', Joy, self.joy_cb, queue_size=1)

    def shutdown():
        print "shutdown time!"
        print "shutdown time!"
        print "shutdown time!"
        self.pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
        return 0

    def unity_cb(self, msg):
        self.target_joint_states.data = [num * RAD2DEG for num in msg.position]
        self.pub_unity.publish(self.target_joint_states)
        print(self.target_joint_states.data)


    #def joy_cb(self, msg):
    #    self.target_joint_states.data = [msg.axes[0]*90.0,0,0,0,0,0]
    #    self.pub_unity.publish(self.target_joint_states)



if __name__ == "__main__":
    snu = SNUDSR()

    while not rospy.is_shutdown():
        pass

    print 'good bye!'