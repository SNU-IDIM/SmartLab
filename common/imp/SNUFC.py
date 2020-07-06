#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
    Python Libraries
'''
import sys, os
import timeit
import math
import numpy as np
import cv2 as cv2
from copy import deepcopy
from math import pi
from time import sleep

'''
    ROS Libraries
'''
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
from tf.transformations import *
from std_msgs.msg import String, Header
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from syscon_msgs.msg import URStatus
from dsr_msgs.msg import RobotState
from ar_track_alvar_msgs.msg import AlvarMarkers

'''
    Doosan-Robot Libraries
'''
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

'''
    Basic Coefficients, Functions (macros)
'''
EPSILON = 0.0000001

def DEG2RAD(degree):
    return (math.pi/180.0)*degree

def RAD2DEG(radian):
    return (180.0/math.pi)*radian

def M2MM(meter):
    return 1000.0*meter

def MM2M(milimeter):
    return (1.0/1000.0)*milimeter


'''
    "~/ur_pnp" Topic Protocol (for Doosan-robot control)
    
    Naming rules:
        1. ACTION_ARM_[이름 정의(대문자)] :   1.0  ~  500.0
        2. TASK_ARM_[이름 정의(대문자)]   : 501.0  ~ 1000.0
'''
## ACTION 정의 (1.0 ~ 500.0)
ACTION_HOME          = '0.0'
ACTION_LEFT_SWING    = '1.0'
ACTION_RIGHT_SWING   = '2.0'
ACTION_POINTING      = '3.0'
ACTION_APPROACH      = '4.0'
ACTION_ENGAGE        = '5.0'


## TASK 정의 (501.0 ~ 1000.0)
TASK_PICK  = '3.0'
TASK_PLACE = '4.0'
TASK_3DP_PICK = '101.0'
PICK_3DP = '3dp'


'''
    Specific Configuration of the Manipulator

    Naming rules:
        1. Joint space coordinate -> Q_[이름(대문자)] = [q0, q1, q2, q3, q4, q5]    [단위: deg]
        2. Task space coordinate  -> P_[이름(대문자)] = [x, y, z, Rz, Ry, Rz]       [단위: mm, deg] 
        3. Q/P_[이름 정의(대문자)] - 위에 정의한 Action/Task와 연관이 있을 경우, 동일한 이름 사용
        4. 되도록이면 숫자 대신 naming 붙일 것 (ex - Q0 (x) // Q_HOME (o))
'''
Q_HOME        = [0.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_LEFT_SWING  = [90.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_RIGHT_SWING = [-90.0, 0.0, -90.0, 0.0, -90.0, 0.0]

Q_TOP_PLATE = [8.031315803527832, 27.553855895996094, -110.20037841796875, 1.7169572114944458, -96.97026062011719, 10.950394630432129]

Q_SEARCH_RIGHT = [-1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_LEFT  = [1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_FRONT = [0.006254249687429258, 0.0706465647310261, -1.8816342308005074, -0.009305934771632234, -0.518931153024292, 0.012760136888951999]

#Q_SEARCH_RIGHT = [-0.021912608043065707, 0.3745233068645807, -2.515318099008636, -0.0016689710660107685, -0.9671584417422292, 0.00014467861565142695]