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


##################################################################################################################################################
'''
    Variables for State Machine (AMR + DSR)
        @ Action Modes (SYSCON 정의 action modes)
        @ DSR Status ("waiting" / "running" / "done")
'''
## Action Modes (SYSCON 정의 Action Mode -> 수정 불가)
SYSCON_WAYPOINT  = 0x01
SYSCON_URMOVEIT  = 0x02
SYSCON_URPNP     = 0x03
SYSCON_URMISSION = 0x05

## DSR Status ("waiting" / "running" / "done")
# DSR_STATUS_WAIT

##################################################################################################################################################



##################################################################################################################################################
'''
    AMR Target Pose Lists

    Naming rules:
        @ DIR_AMR_[방향]: IDIM testbed 기준
        @ P_AMR_[이름(장비 ...)]  : Target Pose [ X[m], Y[m], Theta[rad] ]
'''
DIR_AMR_UP    =  0.0             # 앞쪽 문 방향
DIR_AMR_DOWN  =  math.pi         # 뒷쪽 문 방향
DIR_AMR_LEFT  = -math.pi / 2.0   # Instron 방향 (왼쪽)
DIR_AMR_RIGHT =  math.pi / 2.0   # CNC 방향 (오른쪽)

P_AMR_ZERO      = [ 0.000,   0.000,  DIR_AMR_UP]
P_AMR_HOME      = [ 1.465,  -1.654,  DIR_AMR_LEFT]
P_AMR_DOOR      = [ 7.402,  -0.140,  DIR_AMR_RIGHT]
P_AMR_TABLE_1   = [ 7.071,  -2.000,  DIR_AMR_UP]
P_AMR_TABLE_2   = [ 5.346,  -2.000,  DIR_AMR_UP]
P_AMR_DYNAMO    = [ 4.410,  -2.300,  DIR_AMR_UP]
P_AMR_INSTRON   = [ 2.457,  -2.488,  DIR_AMR_UP]
P_AMR_LOCKER    = [ 3.163,  -0.073,  DIR_AMR_LEFT]
P_AMR_SEWING    = [ 2.061,  -0.078,  DIR_AMR_UP]
P_AMR_CNC       = [ 0.888,  -0.238,  DIR_AMR_UP]
P_AMR_TABLE_3   = [-1.401,  -0.060,  DIR_AMR_DOWN]
P_AMR_JUSTEK    = [-2.917,  -0.590,  DIR_AMR_DOWN]
P_AMR_LASER     = [-4.510,  -0.801,  DIR_AMR_DOWN]
P_AMR_DAEGON    = [-5.571,  -0.777,  DIR_AMR_LEFT]
P_AMR_INJECTION = [-3.575,  -2.460,  DIR_AMR_UP]
P_AMR_PROFILER  = [-1.684,  -2.565,  DIR_AMR_UP]
P_AMR_SINDOH3DP = [-0.134,  -2.565,  DIR_AMR_UP]
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
ACTION_IO_GRIPPER_OPEN       = 101
ACTION_IO_GRIPPER_CLOSE      = 102
ACTION_IO_COMPRESSOR_ON      = 103
ACTION_IO_COMPRESSOR_OFF     = 104
ACTION_IO_TOOLCHANGER_ATTACH = 105
ACTION_IO_TOOLCHANGER_DETACH = 106

## ACTION [X: 1000 ~ 1999 / Y: 2000 ~ 2999 / Z: 3000 ~ 4000] - Translation in X, Y, Z-direction (Relative, +/-)
ACTION_TRANS_X = 1000
ACTION_TRANS_Y = 2000
ACTION_TRANS_Z = 3000
ACTION_TRANS   = 4000

## TASK [10001 ~ 10100] - Simple Task
TASK_SPECIMEN_PICK   = 10001
TASK_INSTRON_SEARCH  = 10002
TASK_INSTRON_MOVEOUT = 10003
TASK_3DP_PICK        = 10004
TASK_3DP_PLACE       = 10005
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
Q_HOME         = [0.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_BACK         = [180.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_LEFT         = [90.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_RIGHT        = [-90.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_TOP_PLATE    = [0.0, 30.0, -110.0, 0.0, -100.0, 0.0]
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