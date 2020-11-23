#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
import sys
import math
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf.transformations import *

NS_ = 'R_001'

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
SI2DSR  = 1000.0

CTRL_SWITCH = 'AMR'
JOY_FLAG = 1
print "Joystick Controller: AMR"

JOY2DSR_TRANS  = 0.001 # [0 ~ 1] -> [m] 
JOY2DSR_ROT    = 1 * DEG2RAD # [0 ~ 1] -> [deg] -> [rad]

AMR_MAX_VEL_LINEAR  = 0.5 # in [m/s]
AMR_MAX_VEL_ANGULAR = 20.0 * DEG2RAD # in [rad/s]
EPSILON = 0.001


BOTTON_A           = 0
BOTTON_B           = 1
BOTTON_X           = 2
BOTTON_Y           = 3
BOTTON_UPPER_LEFT  = 4
BOTTON_UPPER_RIGHT = 5
BOTTON_BACK        = 6
BOTTON_START       = 7
BOTTON_CENTER      = 8
BOTTON_JOY_LEFT    = 9
BOTTON_JOY_RIGHT   = 10

AXIS_LEFT_H        = 0
AXIS_LEFT_V        = 1
AXIS_UPPER_LEFT    = 2
AXIS_RIGHT_H       = 3
AXIS_RIGHT_V       = 4
AXIS_UPPER_RIGHT   = 5
AXIS_DIR_H         = 6
AXIS_DIR_V         = 7


bms = 0.0
amr_cmd = Twist()
dsr_cmd = Joy()



def dsr_shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

    pub_dsr_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def joy_cb(msg):
    global amr_cmd
    global dsr_cmd
    global CTRL_SWITCH
    global JOY_FLAG
    
    if msg.buttons[BOTTON_BACK] == 1 and msg.buttons[BOTTON_START] == 1:
        JOY_FLAG = -JOY_FLAG
        if JOY_FLAG == 1:
            print "Joystick Controller: OFF -> ON"
        else:
            print "Joystick Controller: ON -> OFF"
       
    if msg.buttons[BOTTON_CENTER] != 0:
        if CTRL_SWITCH == 'Manipulator':
            CTRL_SWITCH = 'AMR'
            print "Joystick Controller: Doosan Robot M1013 -> AMR"
        else:
            CTRL_SWITCH = 'Manipulator'
            print "Joystick Controller: AMR -> Doosan Robot M1013"

    if CTRL_SWITCH == 'AMR':
        amr_cmd.linear.x = msg.axes[AXIS_LEFT_V] * AMR_MAX_VEL_LINEAR
	amr_cmd.angular.z = msg.axes[AXIS_LEFT_H] * AMR_MAX_VEL_ANGULAR

        #if abs(amr_cmd.linear.x) < EPSILON:
        #    amr_cmd.angular.z = msg.axes[AXIS_LEFT_H] * AMR_MAX_VEL_ANGULAR
        #else:
        #    amr_cmd.angular.z = msg.axes[AXIS_LEFT_H] * AMR_MAX_VEL_ANGULAR * math.copysign(1, amr_cmd.linear.x)
    
    if CTRL_SWITCH == 'Manipulator':
        dsr_cmd = msg

def bms_cb(msg):
    global bms
    bms = msg.data[3]

def print_joy_state(state):
    print '---'
    print 'Battery State: %.2f/100.00 [%%]'%bms
    print 'Joystick Controller: %s'%state


if __name__ == "__main__":
    rospy.init_node('snu_joystick_controller')
    # Subscribing Topics
    sub_joy  = rospy.Subscriber('/'+NS_+"/joy", Joy, joy_cb)
    sub_bms  = rospy.Subscriber('bms', Float32MultiArray, bms_cb)

    # Publishing Topics
    pub_amr = rospy.Publisher('/'+NS_+'/cmd_vel', Twist, queue_size=10)
    pub_dsr = rospy.Publisher('/'+NS_+'/dsr_cmd', Joy, queue_size=10)
    rate = rospy.Rate(10)
    
    count = 0
    while not rospy.is_shutdown():
        if JOY_FLAG == 1:
            if CTRL_SWITCH is 'AMR':
                if count == 10:
                    print_joy_state('AMR')
                    count = 0
                pub_amr.publish(amr_cmd)
            if CTRL_SWITCH is 'Manipulator':
                if count == 10:
                    print_joy_state('Doosan Robot M1013')
                    count = 0
                pub_dsr.publish(dsr_cmd)
        else:
            if count == 10:
                print_joy_state('Press [START] and [BACK] Bottons for Joystick Manual Control')
                count = 0

        count += 1

        rate.sleep()
        pass

    print 'good bye!'
