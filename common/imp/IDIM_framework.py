#!/usr/bin/env python
# -*- coding: utf-8 -*-

##################################################################################################################################################
'''
    Some Coefficients, Basic Functions (macros)
'''
import math

EPSILON = 0.0000001

def DEG2RAD(degree):
    return (math.pi/180.0)*degree

def RAD2DEG(radian):
    return (180.0/math.pi)*radian

def M2MM(meter):
    return 1000.0*meter

def MM2M(milimeter):
    return (1.0/1000.0)*milimeter
##################################################################################################################################################
'''
    Doosan-robot Default Parameters
'''
DSR_DEFAULT_JOG_VELX = [50, 10]   # [mm/s]
DSR_DEFAULT_JOG_ACCX = [50, 10]   # [mm/s^2]
DSR_DEFAULT_VELX     = [50, 100]  # [mm/s, deg/s]
DSR_DEFAULT_ACCX     = [50, 100]  # [mm/x^2, deg/s^2]
DSR_DEFAULT_VELJ     = 50         # [deg/s]
DSR_DEFAULT_ACCJ     = 50         # [deg/s^2]
##################################################################################################################################################


##################################################################################################################################################
'''
    "~/ur_pnp" Topic Protocol (for Doosan-robot control)
    
    Naming rules:
        @ 세부 목록은 아래 주석으로 명시 후 사용: ex - "## ACTION [101 ~ 200] - Doosan-robot I/O Controller" )
        @ 정의된 숫자 순서에 맞는 위치에 정의 (오름차순)
       
        @ ACTION 과 TASK를 다음과 같이 정의 (새로운 domain (ex - 0 ~ 100)을 사용시 아래 설명 추가를 부탁드립니다)
        @ ACTION_[이름 정의(대문자)] : 0  ~ 10000
            *    0 ~  100: Basic Move
            *  101 ~  200: Doosan-robot I/O Controller
            * 1000 ~ 4000: Relative Move (Translation)
                - X -> 1000 (0 mm) ~ 1999 (999 mm)
                - Y -> 2000 (0 mm) ~ 2999 (999 mm)
                - Z -> 3000 (0 mm) ~ 3999 (999 mm)

        @ TASK_[이름 정의(대문자)]   : 10001  ~ 20000
            * 2개 이상의 Action을 묶을 경우 "TASK" 로 정의
'''
## ACTION [0 ~ 100] - Basic Move
ACTION_HOME         = 0
ACTION_BACK         = 1
ACTION_LEFT         = 2
ACTION_RIGHT        = 3
ACTION_APPROACH     = 4
ACTION_ALIGN        = 5
ACTION_PICK         = 6
ACTION_PLACE        = 7

## ACTION [101 ~ 200] - Doosan-robot I/O Controller
ACTION_IO_COMPRESSOR         =  1   ## DSR Digital I/O 1 -> Compressor
ACTION_IO_COMPRESSOR_ON      =  101
ACTION_IO_COMPRESSOR_OFF     = -101
ACTION_IO_TOOLCHANGER        =  2   ## DSR Digital I/O 2 -> Tool Changer
ACTION_IO_TOOLCHANGER_DETACH =  102
ACTION_IO_TOOLCHANGER_ATTACH = -102
ACTION_IO_JIG_X              =  5   ## DSR Digital I/O 5 -> Universal Jig X-axis
ACTION_IO_JIG_X_CLOSE        =  105
ACTION_IO_JIG_X_OPEN         = -105
ACTION_IO_JIG_Y              =  6   ## DSR Digital I/O 6 -> Universal Jig Y-axis
ACTION_IO_JIG_Y_CLOSE        =  106
ACTION_IO_JIG_Y_OPEN         = -106

ACTION_IO_GRIPPER_CLOSE      =  113 ## Gripper는 임시 -> Flange I/O로 바꿔야 함
ACTION_IO_GRIPPER_OPEN       =  114 ## Gripper는 임시 -> Flange I/O로 바꿔야 함

ACTION_TOOLCHANGE_1_DETACH   =  301
ACTION_TOOLCHANGE_1_ATTACH   = -301
ACTION_TOOLCHANGE_2_DETACH   =  302
ACTION_TOOLCHANGE_2_ATTACH   = -302


## ACTION [X: 1000 ~ 1999 / Y: 2000 ~ 2999 / Z: 3000 ~ 4000] - Translation in X, Y, Z-direction (Relative, +/-)
ACTION_TRANS_X = 1000
ACTION_TRANS_Y = 2000
ACTION_TRANS_Z = 3000
ACTION_TRANS   = 4000

## TASK [10001 ~ 10100] - Simple Task
TASK_SPECIMEN_PICK      = 10001
TASK_INSTRON_SEARCH     = 10002
TASK_INSTRON_MOVEOUT    = 10003
TASK_SEPARATE           = 10004
TASK_SPECIMEN_SEARCH    = 10006
TASK_MULSPECIMEN_SEARCH = 10007

TOOLCHANGE1 =10010
## TASK (Test) [20001 ~ ]
TASK_TEST_COMPLIANCE = 20001
##################################################################################################################################################


##################################################################################################################################################
'''
    Specific Configuration of the Manipulator (for movej, movel)

    Naming rules:
        @ Joint space coordinate -> Q_[이름(대문자)] = [q0, q1, q2, q3, q4, q5]    [단위: deg]
        @ Task space coordinate  -> P_[이름(대문자)] = [x, y, z, Rz, Ry, Rz]       [단위: mm, deg] 
        @ Q/P_[이름 정의(대문자)] - 위에 정의한 Action/Task와 연관이 있을 경우, 동일한 이름 사용
        @ 되도록이면 숫자 대신 naming 붙일 것 (ex - Q0 (x) // Q_HOME (o))
'''
## Joint Space Coordinates (Q)
Q_HOME      = [0.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_BACK      = [180.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_LEFT      = [90.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_RIGHT     = [-90.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_TOP_PLATE = [0.0, 30.0, -110.0, 0.0, -100.0, 0.0]
Q_SEARCH_RIGHT = [-1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_LEFT  = [1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_FRONT = [0.006254249687429258, 0.0706465647310261, -1.8816342308005074, -0.009305934771632234, -0.518931153024292, 0.012760136888951999]

## Task Space Coordinates (P)
P_TOP_PLATE = [-240.5, 34.5, 665.5, 0.0, 180.0, 0.0] # Same configuration with "Q_TOP_PLATE"
##################################################################################################################################################


##################################################################################################################################################
'''
    Parameters for Joystick Controller
'''
JOY_MAX_SPEED_LINEAR  = 100.0
JOY_MAX_SPEED_ANGULAR = 100.0

JOY_BOTTON_A           = 0
JOY_BOTTON_B           = 1
JOY_BOTTON_X           = 2
JOY_BOTTON_Y           = 3
JOY_BOTTON_UPPER_LEFT  = 4
JOY_BOTTON_UPPER_RIGHT = 5
JOY_BOTTON_BACK        = 6
JOY_BOTTON_START       = 7
JOY_BOTTON_CENTER      = 8
JOY_BOTTON_JOY_LEFT    = 9
JOY_BOTTON_JOY_RIGHT   = 10

JOY_AXIS_LEFT_H        = 0
JOY_AXIS_LEFT_V        = 1
JOY_AXIS_UPPER_LEFT    = 2
JOY_AXIS_RIGHT_H       = 3
JOY_AXIS_RIGHT_V       = 4
JOY_AXIS_UPPER_RIGHT   = 5
JOY_AXIS_DIR_H         = 6
JOY_AXIS_DIR_V         = 7