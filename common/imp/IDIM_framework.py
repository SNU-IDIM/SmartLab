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
    "~/ur_pnp" Topic Protocol (for Doosan-robot control)
    
    Naming rules:
        @ 세부 목록은 아래 주석으로 명시 후 사용: ex - "## ACTION [101.0 ~ 200.0] - Doosan-robot I/O Controller" )
        @ 설정 값은 정수로 인식되지만, 아래 정의할 때엔 String type으로 정의 (ex - 1.0 (x) // '1.0' (o) // '1.1' (x))
        @ 정의된 숫자 순서에 맞는 위치에 정의 (오름차순)
       
        @ ACTION 과 TASK를 다음과 같이 정의 (새로운 domain (ex - 0.0 ~ 100.0)을 사용시 아래 설명 추가를 부탁드립니다)
        @ ACTION_[이름 정의(대문자)] : 0.0  ~ 10000.0
            *    0.0 ~  100.0: Basic Move
            *  101.0 ~  200.0: Doosan-robot I/O Controller
            * 1000.0 ~ 4000.0: Relative Move (Translation)
                - X -> 1000.0 (0 mm) ~ 1999.0 (999 mm)
                - Y -> 2000.0 (0 mm) ~ 2999.0 (999 mm)
                - Z -> 3000.0 (0 mm) ~ 3999.0 (999 mm)

        @ TASK_[이름 정의(대문자)]   : 10001.0  ~ 20000.0
            * 2개 이상의 Action을 묶을 경우 "TASK" 로 정의
'''
## ACTION [0.0 ~ 100.0] - Basic Move
ACTION_HOME         = '0.0'
ACTION_BACK         = '1.0'
ACTION_LEFT         = '2.0'
ACTION_RIGHT        = '3.0'
ACTION_APPROACH     = '4.0'
ACTION_ALIGN        = '5.0'
ACTION_PICK         = '6.0'
ACTION_PLACE        = '7.0'

## ACTION [101.0 ~ 200.0] - Doosan-robot I/O Controller
ACTION_IO_GRIPPER_OPEN       = '101.0'
ACTION_IO_GRIPPER_CLOSE      = '102.0'
ACTION_IO_COMPRESSOR_ON      = '103.0'
ACTION_IO_COMPRESSOR_OFF     = '104.0'
ACTION_IO_TOOLCHANGER_ATTACH = '105.0'
ACTION_IO_TOOLCHANGER_DETACH = '106.0'

## ACTION [1000.0 ~  1999.0] - Translation in X-direction (Relative, +/-)
ACTION_TRANS_X = '1000.0'
## ACTION [2000.0 ~  2999.0] - Translation in Y-direction (Relative, +/-)
ACTION_TRANS_Y = '2000.0'
## ACTION [3000.0 ~  4000.0] - Translation in X-direction (Relative, +/-)
ACTION_TRANS_Z = '3000.0'
ACTION_TRANS   = '4000.0'

## TASK [10001.0 ~ 10100.0] - Simple Task
TASK_SPECIMEN_PICK = '10001.0'
TASK_PLACE         = '10002.0'
TASK_3DP_PICK      = '10003.0'
TASK_3DP_PLACE     = '10003.0'
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