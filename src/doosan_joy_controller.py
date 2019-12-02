#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
import sys
from sensor_msgs.msg import Joy
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/doosan-robot/common/imp"%HOME_DIR)) )
 # get import pass : DSR_ROBOT.py 

# for single robot
NS_           = "R_001"
ROBOT_ID_     = "dsr"
ROBOT_MODEL_  = ""
import DR_init
CountThreshold = 10
m_joyAnalogFlag = False
m_TxyCompareFlag = False
m_RxyCompareFlag = False
m_joyButtonFlag = False
m_joyJogFlag = 0
m_joyJogVel = False
compressor_flag = 0
compressor_count = 0
toolchanger_flag = 0
toolchanger_count = 0
DR_init.__dsr__id = NS_+'/'+ROBOT_ID_
DR_init.__dsr__model = ROBOT_MODEL_
from DSR_ROBOT import *

MAX_ROBOT_SPEED_LINEAR  = 100.0
MAX_ROBOT_SPEED_ANGULAR = 100.0
ROBOT_SPEED_LINEAR      = 50.0 # in [mm/s]
ROBOT_SPEED_ANGULAR     = 50.0 # in [deg/s]
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


def shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        #print("  current_posx      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2],msg.current_posx[3],msg.current_posx[4],msg.current_posx[5]))

        #print("  io_control_box    : %d" % (msg.io_control_box))
        ##print("  io_modbus         : %d" % (msg.io_modbus))
        ##print("  error             : %d" % (msg.error))
        #print("  access_control    : %d" % (msg.access_control))
        #print("  homming_completed : %d" % (msg.homming_completed))
        #print("  tp_initialized    : %d" % (msg.tp_initialized))
        #print("  speed             : %d" % (msg.speed))
        #print("  mastering_need    : %d" % (msg.mastering_need))
        #print("  drl_stopped       : %d" % (msg.drl_stopped))
        #print("  disconnected      : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber(ROBOT_ID_ +ROBOT_MODEL_+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    -

r = CDsrRobot(NS_+'/'+ROBOT_ID_, ROBOT_MODEL_)

def joy_cb(msg):
    global m_joyAnalogFlag  # Global flag
    global m_TxyCompareFlag
    global m_RxyCompareFlag 
    global m_joyButtonFlag
    global m_joyJogFlag     # Jog flag
    global m_joyJogVel
    global ROBOT_SPEED_LINEAR
    global ROBOT_SPEED_ANGULAR
    global compressor_flag
    global compressor_count
    global toolchanger_flag
    global toolchanger_count

    targetPos = [0, 0, -90, 0, -90, 0]
    hommingPos = [0, 0, 0, 0, 0, 0]


    if msg.buttons[BOTTON_START] == 1:
        r.movej(targetPos, 30, 50)
    elif msg.buttons[BOTTON_BACK] == 1:
        r.movej(hommingPos, 30, 50)

    if msg.buttons[BOTTON_UPPER_LEFT] == 1:
        pub_pnp.publish('open')
    if msg.buttons[BOTTON_UPPER_RIGHT] == 1:
        pub_pnp.publish('close')

    if msg.buttons[BOTTON_X] == 1:
        compressor_count += 1
        if compressor_count > CountThreshold and compressor_flag == 0:
            pub_pnp.publish('compressor on')
            compressor_count = 0
            compressor_flag = 1
        elif compressor_count > CountThreshold and compressor_flag == 1:
            pub_pnp.publish('compressor off')
            compressor_count = 0
            compressor_flag = 0

    if msg.buttons[BOTTON_Y] == 1:
        compressor_count += 1
        if compressor_count > CountThreshold and compressor_flag == 0:
            pub_pnp.publish('toolchanger detach')
            compressor_count = 0
            compressor_flag = 1
        elif compressor_count > CountThreshold and compressor_flag == 1:
            pub_pnp.publish('toolchanger attach')
            compressor_count = 0
            compressor_flag = 0   

    if msg.buttons[BOTTON_B] == 1:
        ROBOT_SPEED_LINEAR  += 10
        if ROBOT_SPEED_LINEAR >= MAX_ROBOT_SPEED_LINEAR: ROBOT_SPEED_LINEAR = MAX_ROBOT_SPEED_LINEAR
        ROBOT_SPEED_ANGULAR += 10
        if ROBOT_SPEED_ANGULAR >= MAX_ROBOT_SPEED_ANGULAR: ROBOT_SPEED_ANGULAR = MAX_ROBOT_SPEED_ANGULAR
    
    if msg.buttons[BOTTON_A] == 1:
        ROBOT_SPEED_LINEAR  -= 10
        if ROBOT_SPEED_LINEAR <= 0.0: ROBOT_SPEED_LINEAR = 0.0
        ROBOT_SPEED_ANGULAR -= 10
        if ROBOT_SPEED_ANGULAR >= 0.0: ROBOT_SPEED_ANGULAR = 0.0

    
    # Analog 신호 하나라도 들어오면 m_joyAnalogFlag -> set 됨 (global flag)
    if msg.axes[AXIS_LEFT_V] != 0 or msg.axes[AXIS_LEFT_H] != 0 or msg.axes[AXIS_RIGHT_V] != 0 or msg.axes[AXIS_RIGHT_H] != 0 or msg.axes[AXIS_DIR_V] != 0 or msg.axes[AXIS_DIR_H] != 0:
        m_joyAnalogFlag = True
    else:
        m_joyAnalogFlag = False

    # 왼쪽 joystick -> m_TxyCompareFlag 로 구분
    if msg.axes[AXIS_LEFT_V] != 0 or msg.axes[AXIS_LEFT_H] or 0:
        if abs(msg.axes[AXIS_LEFT_V]) > abs(msg.axes[AXIS_LEFT_H]):
            m_TxyCompareFlag = False
        else:
            m_TxyCompareFlag = True
        
    # 오른쪽 joystick -> m_RxyCompareFlag 로 구분
    if msg.axes[AXIS_RIGHT_V] != 0 or msg.axes[AXIS_RIGHT_H] or 0:
        if abs(msg.axes[AXIS_RIGHT_V]) > abs(msg.axes[AXIS_RIGHT_H]):
            m_RxyCompareFlag = False
        else:
            m_RxyCompareFlag = True

    # 왼쪽 방향 버튼 -> m_ZCompareFlag 로 구분
    if msg.axes[AXIS_DIR_V] != 0 or msg.axes[AXIS_DIR_H] != 0:
        m_ZCompareFlag = True
    else:
        m_ZCompareFlag = False



    ### ANALOG 제어
    if m_joyJogFlag == -1 and not m_joyButtonFlag and m_joyAnalogFlag: # m_joyAnalogFlag에 대한 움직임
        if m_ZCompareFlag:
            if msg.axes[AXIS_DIR_H] == 1:
                m_joyJogFlag = JOG_AXIS_TASK_RZ
                m_joyJogVel = -ROBOT_SPEED_LINEAR
            if msg.axes[AXIS_DIR_H] == -1:
                m_joyJogFlag = JOG_AXIS_TASK_RZ
                m_joyJogVel = ROBOT_SPEED_LINEAR

            # AXIS_DIR_LEFT/RIGHT -> Z-direction Translation
            if msg.axes[AXIS_DIR_V] == 1:
                m_joyJogFlag = JOG_AXIS_TASK_Z
                m_joyJogVel = -ROBOT_SPEED_LINEAR
            if msg.axes[AXIS_DIR_V] == -1:
                m_joyJogFlag = JOG_AXIS_TASK_Z
                m_joyJogVel = ROBOT_SPEED_LINEAR


        # AXIS_LEFT_UP/DOWN -> X-direction Translation
        if msg.axes[AXIS_LEFT_V] > 0 and m_TxyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_X
            m_joyJogVel = ROBOT_SPEED_LINEAR
        if msg.axes[AXIS_LEFT_V] < 0 and m_TxyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_X
            m_joyJogVel = -ROBOT_SPEED_LINEAR

        # AXIS_LEFT_LEFT/RIGHT -> Y-direction Translation
        if msg.axes[AXIS_LEFT_H] > 0 and m_TxyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel = -ROBOT_SPEED_LINEAR
        if msg.axes[AXIS_LEFT_H] < 0 and m_TxyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel = ROBOT_SPEED_LINEAR

        # AXIS_RIGHT_UP/DOWN -> Y-direction Rotation
        if msg.axes[AXIS_RIGHT_V] > 0 and m_RxyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_RY
            m_joyJogVel = -ROBOT_SPEED_LINEAR
        if msg.axes[AXIS_RIGHT_V] < 0 and m_RxyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_RY
            m_joyJogVel = ROBOT_SPEED_LINEAR

        # AXIS_RIGHT_LEFT/RIGHT -> X-direction Rotation
        if msg.axes[AXIS_RIGHT_H] > 0 and m_RxyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_RX
            m_joyJogVel = -ROBOT_SPEED_LINEAR
        if msg.axes[AXIS_RIGHT_H] < 0 and m_RxyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_RX
            m_joyJogVel = ROBOT_SPEED_LINEAR

        r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, m_joyJogVel)

    else:
        if not m_joyAnalogFlag and not m_joyButtonFlag:
            rospy.loginfo("jog stop")
            r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, 0)
            m_joyJogFlag = -1


if __name__ == "__main__":
    rospy.init_node('dsr_joystick_control')
    rospy.on_shutdown(shutdown)

    #t1 = threading.Thread(target=thread_subscriber)
    #t1.daemon = True 
    #t1.start()

    pub_stop = rospy.Publisher(ROBOT_ID_ +ROBOT_MODEL_+'/stop', RobotStop, queue_size=1)
    pub_pnp = rospy.Publisher('ur_pnp', String, queue_size=1)
    sub_joy  = rospy.Subscriber("dsr_cmd", Joy, joy_cb)
    while not rospy.is_shutdown():
        pass

    print 'good bye!'


