#!/usr/bin/env python
# -*- coding: utf-8 -*-

##################################################################################################################################################
'''
    Some Coefficients, Basic Functions (macros)
'''
import math
from copy import deepcopy

EPSILON = 0.0001

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
    DOE Types
'''
DOE_TWO_LEVEL_FULL_FACTORIAL       = 1
DOE_THREE_LEVEL_FULL_FACTORIAL     = 2
DOE_GENERALIZED_FACTORIAL          = 3
DOE_TWO_LEVEL_FRACTIONAL_FACTORIAL = 4
DOE_PLACKETT_BURMAN                = 5
DOE_BOX_BEHNKEN                    = 6
DOE_CENTRAL_COMPOSITE              = 7

##################################################################################################################################################
'''
    Current End-effector number (Tool changer)
'''
EEF_NONE  = 0
EEF_TOOL1 = 1
EEF_TOOL2 = 2
EEF_TOOL3 = 3
EEF_TOOL4 = 4

'''
    Doosan-robot Default Parameters
'''
DSR_DEFAULT_JOG_VELX = [50, 10]   # [mm/s]
DSR_DEFAULT_JOG_ACCX = [50, 10]   # [mm/s^2]
DSR_DEFAULT_VELX     = [50, 100]  # [mm/s, deg/s]
DSR_DEFAULT_ACCX     = [50, 100]  # [mm/x^2, deg/s^2]
DSR_DEFAULT_VELJ     = 50         # [deg/s]
DSR_DEFAULT_ACCJ     = 50         # [deg/s^2]

'''
    AR target Default Parameters (Offset from AR_marker)
'''
DEFAULT_OFFSET_FROM_TARGET_X  = 0.0     ## unit in [m]
DEFAULT_OFFSET_FROM_TARGET_Y  = 0.0     ## unit in [m]
DEFAULT_OFFSET_FROM_TARGET_Z  = 0.175   ## unit in [m]
DEFAULT_OFFSET_FROM_TARGET_RX = 180.0   ## unit in [deg]
DEFAULT_OFFSET_FROM_TARGET_RY = 0.0     ## unit in [deg]
DEFAULT_OFFSET_FROM_TARGET_RZ = -90.0    ## unit in [deg]
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
AMR_DIR_UP    =  0.0             # Instron 방향 (왼쪽)
AMR_DIR_DOWN  =  math.pi         # 뒷쪽 문 방향
AMR_DIR_LEFT  = -math.pi / 2.0   # CNC 방향 (오른쪽)
AMR_DIR_RIGHT =  math.pi / 2.0   # 앞쪽 문 방향


AMR_POS_HOME      = [ 0.000,  0.000,  0.000]
AMR_POS_3DP_0     = [ 1.037, -2.793, -1.560];   AMR_OFFSET_3DP = 0.4676
AMR_POS_INSTRON   = [ 0.928, -2.835, -1.570]
AMR_POS_OMM       = [ 0.804,  1.621,  1.617]


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

## ACTION [101 ~ 200] - Doosan-robot I/O Controller (DSR Controller)
ACTION_IO_COMPRESSOR         =  1   ## DSR Digital I/O 1 -> Compressor
ACTION_IO_COMPRESSOR_ON      =  101
ACTION_IO_COMPRESSOR_OFF     = -101
ACTION_IO_TOOLCHANGER        =  2   ## DSR Digital I/O 2 -> Tool Changer
ACTION_IO_TOOLCHANGER_DETACH =  102
ACTION_IO_TOOLCHANGER_ATTACH = -102
ACTION_IO_TBD                =  3   ## DSR Digital I/O 3
ACTION_IO_TBD                =  103
ACTION_IO_TBD                = -103
# ACTION_IO_TBD               =  4   ## DSR Digital I/O 4
# ACTION_IO_TBD               =  104
# ACTION_IO_TBD               = -104
ACTION_IO_JIG_X              =  5   ## DSR Digital I/O 5 -> Universal Jig X-axis
ACTION_IO_JIG_X_CLOSE        =  105
ACTION_IO_JIG_X_OPEN         = -105
ACTION_IO_JIG_Y              =  6   ## DSR Digital I/O 6 -> Universal Jig Y-axis
ACTION_IO_JIG_Y_CLOSE        =  106
ACTION_IO_JIG_Y_OPEN         = -106
# ACTION_IO_TBD               =  7   ## DSR Digital I/O 7
# ACTION_IO_TBD               =  107
# ACTION_IO_TBD               = -107
ACTION_IO_SUCTIONCUP         =  8   ## DSR Digital I/O 8 -> Suction Cup Gripper (Negative pressure)
ACTION_IO_SUCTIONCUP_ON      =  108
ACTION_IO_SUCTIONCUP_OFF     = -108

## ACTION [201 ~ 300] - Doosan-robot I/O Controller (DSR Flange)
ACTION_IO_GRIPPER_CLOSE      = -201
ACTION_IO_GRIPPER_OPEN       =  201

## ACTION [301 ~ 400] - Fundamental Actions (for Setting the Robot)
ACTION_TOOLCHANGE_1_DETACH   = -301
ACTION_TOOLCHANGE_1_ATTACH   =  301
ACTION_TOOLCHANGE_2_DETACH   = -302
ACTION_TOOLCHANGE_2_ATTACH   =  302


## ACTION [X: 1000 ~ 1999 / Y: 2000 ~ 2999 / Z: 3000 ~ 4000] - Translation in X, Y, Z-direction (Relative, +/-)
ACTION_TRANS_X = 1000
ACTION_TRANS_Y = 2000
ACTION_TRANS_Z = 3000
ACTION_TRANS   = 4000

## TASK [10001 ~ 10100] - Simple Task
TASK_SPECIMEN_PICK        =  10001
TASK_INSTRON_SEARCH       =  10002
TASK_INSTRON_MOVEOUT      =  10003
TASK_DETACH_SPECIMEN      =  10004 
TASK_SEARCH_PICK_SPECIMEN =  10005
TASK_SPECIMEN_TO_RACK     =  10006
TASK_SPECIMEN_FROM_RACK   =  10007
TASK_RACK_ALIGN           =  10008
TASK_INSTRON_CLEAN        =  10009


TASK_3DP_BED_IN           =  10010 ## 실제로는 비어있음 (여기에 +숫자 해서 사용)
TASK_3DP_BED_OUT          = -10010 ## 실제로는 비어있음 (여기에 -숫자 해서 사용)
TASK_3DP_1_BED_IN         =  10011
TASK_3DP_1_BED_OUT        = -10011
TASK_3DP_2_BED_IN         =  10012
TASK_3DP_2_BED_OUT        = -10012
TASK_3DP_3_BED_IN         =  10013
TASK_3DP_3_BED_OUT        = -10013
TASK_3DP_4_BED_IN         =  10014
TASK_3DP_4_BED_OUT        = -10014
TASK_ATTACH_SENSOR        =  10020  ## ACTION [10020] - Demo // Action [10021 ~ 10028] - Attach sensor (1~8) to specimen


TASK_ADHESIVE_SAVER_IN    =  10031
TASK_ADHESIVE_SAVER_OUT   = -10031
TASK_ADHESIVE_DROP        =  10032
TASK_SEPCIMEN_TO_CENTER   =  10033
TASK_SPECIMEN_TO_LEFT     =  10034
TASK_SPECIMEN_TO_RIGHT    =  10035
TASK_SPECIMEN_READY       =  10036


## TBD... [20001 ~ ]
TASK_TEST_COMPLIANCE      = 20001

## 2D VISION FLAG [30001 ~ ]
VISION_2D_SEARCH_SPECIMEN = 30001
TASK_JOG_DEVEL            = 30002

## 기계시스템설계 Task (100001 ~)
MSD_YJ_1 = 100001



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
Q_HOME                 = [0.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_BACK                 = [180.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_LEFT                 = [90.0, 0.0, -90.0, 0.0, -90.0, 0.0]
Q_RIGHT                = [-90.0, 0.0, -90.0, 0.0, -90.0, 0.0]
# Q_TOP_PLATE            = [0.0, 30.0, -110.0, 0.0, -100.0, 180.0]
Q_TOP_PLATE            = [0.0, 30.0, -110.0, 0.0, -100.0, -180] #티칭 수정 21.04.06
Q_SEARCH_RIGHT         = [-1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_LEFT          = [1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_FRONT         = [0.006254249687429258, 0.0706465647310261, -1.8816342308005074, -0.009305934771632234, -0.518931153024292, 0.012760136888951999]
Q_MULSPECIMEN_SEARCH   = [-19.77288246154785, 5.743539333343506, -131.46726989746094, -0.0, -54.27621841430664, 161.5270538330078-360]
Q_SEARCH_3DP_PLATE     = [-4.7292633056640625, -2.0135130882263184, -124.34613037109375, 0.9737170934677124, -53.63304138183594, 129.6439666748047]
Q_UNIVERSALJIG_3DP_BED = [-4.624578952789307, -9.224052429199219, -130.3548126220703, 2.0381743907928467, -39.747581481933594, 38.829227447509766]
# Q_SEARCH_3DP_RIGHT     = [-103.04147338867188, -6.241026401519775, -128.091552734375, 0.265263170003891, -42.99394607543945, 163.57440185546875]
Q_SEARCH_3DP_RIGHT     = [-89.74018096923828, 31.135026931762695, -118.15650177001953, -0.22129933536052704, -55.051727294921875, 181.1175994873047]
Q_COLOR_SENSOR_TRAY_RETRACT = [-14.852486610412598, 11.348645210266113, -140.4371795654297, -0.0, -50.91157913208008, -104.8525161743164]
Q_PLACE_INITIAL        = [25.24275016784668, 14.780960083007812, -138.33627319335938, -0.0, -56.44465637207031, -154.7573699951172]
Q_PLACE_INCLINE        = [45.565223693847656, -7.679841995239258, -139.80552673339844, -31.563058853149414, -71.04197692871094, 336.5533752441406-360]
Q_PICK_INCLINE         = [-26.08780288696289, 12.274105072021484, -136.6134796142578, -0.0, -55.66070556640625, 153.91213989257812-360]
## Task Space Coordinates (P)
P_TOP_PLATE            = [-240.5, 34.5, 665.5, 0.0, 180.0, 0.0] ## Same configuration with "Q_TOP_PLATE"
P_TOOLCHANGE_1         = [-436.7486267089844, -347.74090576171875, 69.52918243408203, 100.41868591308594, 178.5112762451172, 21.856922149658203] ## Toolchange position 1
P_TOOLCHANGE_2         = [-277.34544677734375, -345.6324890136719, 69.73489837646484, 112.78761291503906, 179.604248046875, 34.068626403808594]  ## Toolchange position 2
P_MULSPECIMEN_SEARCH   = [-348.00201416015625, 147.00033569335938, 379.99737548828125, 162.33998107910156, -179.99990844726562, -17.659982681274414]
P_UNIVERSALJIG_3DP_BED = [-461.6261901855469, 51.8516845703125, 272.7024841308594, 91.4110336303711, 175.78863525390625, -131.13372802734375]
# P_UNIVERSALJIG_3DP_BED = [-440.0015869140625, 52.0435791015625, 225.61996459960938, 0.5350197553634644, -178.6567840576172, -136.9725341796875]
P_SEARCH_3DP_1_RIGHT   = [70.47772979736328, 395.1345520019531, 409.1728210449219, 47.5877685546875, 177.3722381591797, -39.428524017333984]

# Color sensor tray related position
P_COLOR_SENSOR_TRAY_LEFT  = [-323.95306396484375, 122.93646240234375, 186.5114288330078, 90, -180, -90]
P_COLOR_SENSOR_TRAY_RIGHT = [-229.8754638671875, 126.66502380371094, 186.5114288330078, 180, -180, 180]
OFFSET_COLOR_SENSOR = 17.0
P_COLOR_SENSOR_1 = deepcopy(P_COLOR_SENSOR_TRAY_LEFT);   P_COLOR_SENSOR_1[1] -=  2 * OFFSET_COLOR_SENSOR 
P_COLOR_SENSOR_2 = deepcopy(P_COLOR_SENSOR_TRAY_LEFT);   P_COLOR_SENSOR_2[1] -=  1 * OFFSET_COLOR_SENSOR 
P_COLOR_SENSOR_3 = deepcopy(P_COLOR_SENSOR_TRAY_LEFT);   P_COLOR_SENSOR_3[1] -= -1 * OFFSET_COLOR_SENSOR
P_COLOR_SENSOR_4 = deepcopy(P_COLOR_SENSOR_TRAY_LEFT);   P_COLOR_SENSOR_4[1] -= -2 * OFFSET_COLOR_SENSOR
P_COLOR_SENSOR_5 = deepcopy(P_COLOR_SENSOR_TRAY_RIGHT);  P_COLOR_SENSOR_5[1] -=  2 * OFFSET_COLOR_SENSOR 
P_COLOR_SENSOR_6 = deepcopy(P_COLOR_SENSOR_TRAY_RIGHT);  P_COLOR_SENSOR_6[1] -=  1 * OFFSET_COLOR_SENSOR 
P_COLOR_SENSOR_7 = deepcopy(P_COLOR_SENSOR_TRAY_RIGHT);  P_COLOR_SENSOR_7[1] -= -1 * OFFSET_COLOR_SENSOR
P_COLOR_SENSOR_8 = deepcopy(P_COLOR_SENSOR_TRAY_RIGHT);  P_COLOR_SENSOR_8[1] -= -2 * OFFSET_COLOR_SENSOR

# RACK PICK AND PLACE POSITION
P_AFTER_ATTACH = [-260.0, -182.0, 210, 28.504039764404297, 177.5694580078125, 28.337604522705078]

P_PLACE_INITIAL = [-293.0, -104.0, 322.0, -180.0, -180.0, 0.0]
P_PLACE_INCLINE = [-293.0, -164.0, 210.0, 90.0, 135.0, 0.0]
P_PLACE_RACK_1  = [-293.0, -77.75, 191.0, 90.0, 135.0, 0.0]
P_PLACE_RACK_2  = deepcopy(P_PLACE_RACK_1);                      P_PLACE_RACK_2[1] -= 26.49; 
P_PLACE_RACK_3  = deepcopy(P_PLACE_RACK_1);                      P_PLACE_RACK_3[1] -= 26.49 * 2.0; 
P_PLACE_RACK_4  = deepcopy(P_PLACE_RACK_1);                      P_PLACE_RACK_4[1] -= 26.49 * 3.0; 
P_PICK_INITIAL  = [-281.0, 176.0, 322.0, -180.0, -180.0, 0.0]
P_PICK_INCLINE  = [-281.0, 176.0, 200.0, 90.0, -135.0, 0.0]
P_PICK_RACK_1   = [-281.0, 167.2, 178.2, 90.0, -135.0, 0.0]
P_PICK_RACK_2   = deepcopy(P_PICK_RACK_1);                       P_PICK_RACK_2[1] -= 26.49; 
P_PICK_RACK_3   = deepcopy(P_PICK_RACK_1);                       P_PICK_RACK_3[1] -= 26.49 * 2.0; 
P_PICK_RACK_4   = deepcopy(P_PICK_RACK_1);                       P_PICK_RACK_4[1] -= 26.49 * 3.0; 



##################################################################################################################################################


##################################################################################################################################################
'''
    Parameters for Joystick Controller
'''
JOY_MAX_SPEED_LINEAR   = 100.0
JOY_MAX_SPEED_ANGULAR  = 100.0

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

IDIM_TEST_GRIP           =100000
IDIM_TEST_OPEN           =200000

IDIM_CONTROL_TEST      = 30000
