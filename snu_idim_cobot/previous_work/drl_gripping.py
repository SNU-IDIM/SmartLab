#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys, os
import numpy as np
import moveit_commander
import timeit
from math import pi
from time import sleep
from copy import deepcopy
import math

import moveit_msgs.msg
from std_msgs.msg import String, Header
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from syscon_msgs.msg import URStatus
from dsr_msgs.msg import RobotState
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from tf.transformations import *
import cv2 as cv


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



TASK_POINTING  = 'pointing'
TASK_ARM_PICK  = '3.0'
TASK_ARM_PLACE = '4.0'
PICK_3DP = '3dp'


# Some of Contstants
DISTANCE_AWAY_FROM_TARGET = 0.2

EPSILON = 0.0000001
# DEG2RAD = math.pi / 180.0
DEG2RAD = 1

RAD2DEG = 180.0 / math.pi
M2MM = 1000.0
MM2M = 1.0/1000.0

ROLL = 180.0 * DEG2RAD
PITCH = 0.0 * DEG2RAD
YAW = 0.0 * DEG2RAD

Q0 = [0.0,    0.0,    -90.0,    0.0,    -90.0,    0.0]
Q0 = [8.031315803527832, 27.553855895996094, -110.20037841796875, 1.7169572114944458, -96.97026062011719, 10.950394630432129]
Q_SEARCH_RIGHT = [-1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_LEFT  = [1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_FRONT = [0.006254249687429258, 0.0706465647310261, -1.8816342308005074, -0.009305934771632234, -0.518931153024292, 0.012760136888951999]

#Q_SEARCH_RIGHT = [-0.021912608043065707, 0.3745233068645807, -2.515318099008636, -0.0016689710660107685, -0.9671584417422292, 0.00014467861565142695]

target_pose = Pose()
eulerZYZ = np.zeros(3)

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True



def dsr_state_cb(self, data):
    
    global dsr_flag
    dsr_flag = data.robot_state

def current_status_cb(self, data):
    self.joints_state = data


def quat2Euler(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    print(roll)
    print('cosroll' ,math.cos(roll))

    
    return roll, pitch, yaw


def Zrot(z):
    print '\n'
    print(z)
    print '\n'
    print('math.cos(z)', math.cos(z))
    rot_matrix = np.array([[math.cos(z), -math.sin(z), 0],[math.sin(z),math.cos(z),0],[0,0,1]])
    return rot_matrix

def Yrot(z):
    rot_matrix = np.array([[math.cos(z),0, math.sin(z)],[0,1,0],[-math.sin(z),0,math.cos(z)]])
    return rot_matrix
def Xrot(z):
    rot_matrix = np.array([[1,0,0], [0, math.cos(z), -math.sin(z)],[0, math.sin(z),math.cos(z)]])
    return rot_matrix



def EulerZYZtransformer(r):
    euler = np.zeros(3)
    if(r[2][2] <+1):
        if(r[2][2] >-1):
            euler[0] = math.atan2(r[1][2],r[0][2])
            euler[1] = math.acos(r[2][2])
            euler[2] = math.atan2(r[2][1],-r[2][0])
        ##r22 =1 case
        else:
            euler[0] =-math.atan2(r[1][0],r[1][1])
            euler[1] =math.pi
            euler[2] =0
    else: ##if r22 is +1 cos
        euler[0]= math.atan2(r[1][0],r[1][1])
        euler[1]= 0
        euler[2]= 0
    return euler


def camera2global(x,y,z):
    return Yrot(90*DEG2RAD)

def update_cmd_pose(trans, rot):
    global target_pose
    target_pose.position.x    = trans[0] * 1000 # 보정 @(arm -> 측면)
    target_pose.position.y    = trans[1] * 1000 # 보정 @(arm -> 정면)
    target_pose.position.z    = trans[2] * 1000
    target_pose.orientation.x = rot[0]
    target_pose.orientation.y = rot[1]
    target_pose.orientation.z = rot[2]
    target_pose.orientation.w = rot[3]
    print(target_pose)


# def cmd_moveit_cb():
#     global target_pose
#     target_frame_name = 'ar_marker_1'
#     reference_frame_name = 'base_0'
    
#     listener = tf.TransformListener()

#     try:
#         print "Trying to search the target: %s ..."%target_frame_name
#         listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(1.0))
#         (trans,rot) = listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
#         update_cmd_pose(trans, rot)
#         print(target_pose)

#     except (Exception):
#         print "[ERROR]: The Target(TF) is not Detected !!!"
#         pass

#     # target_pose.position.x = msg.position.x*1000
#     # target_pose.position.y = msg.position.y*1000
#     # target_pose.position.z = msg.position.z*1000
#     # ##camera transform
#     # target_pose.orientation.x = msg.orientation.x
#     # target_pose.orientation.y = msg.orientation.y
#     # target_pose.orientation.z = msg.orientation.z
#     # target_pose.orientation.w = msg.orientation.w

#     roll, pitch, yaw = quat2Euler(target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w)
    
#     zrot = Zrot(roll)
#     yrot = Yrot(pitch)
#     xrot = Xrot(yaw)

#     T = zrot.dot(yrot).dot(xrot)
#     global eulerZYZ
#     eulerZYZ = EulerZYZtransformer(T)
#     print('euler angle for ZYZ is', eulerZYZ)
    
def quat2rot(x,y,z,w):
    r11 = 2*(y*z -w*x)
    r12 = 2*(x*z +w*y)
    r21 = w*w-x*x+z*z
    r31 = 2*(y*z +w*x)
    r32 = -2*(x*z-w*y)
    return r11, r12, r21, r31, r32

def rot2eulzyz(r11, r12,r21,r31,r32):
    z1 = math.atan2(r11,r12)
    y1 = math.acos(r21)
    z2 = math.atan2(r31,r32)
    return z1, y1, z2

def cmd_moveit_cb():
    global target_pose
    target_frame_name = 'ar_target_1'
    reference_frame_name = 'base_0'
    
    listener = tf.TransformListener()

    try:
        print "Trying to search the target: %s ..."%target_frame_name
        listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(1.0))
        (trans,rot) = listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
        update_cmd_pose(trans, rot)
        print('target pose is ' , target_pose)

    except (Exception):
        print "[ERROR]: The Target(TF) is not Detected !!!"
        pass

    # target_pose.position.x = msg.position.x*1000
    # target_pose.position.y = msg.position.y*1000
    # target_pose.position.z = msg.position.z*1000
    # ##camera transform
    # target_pose.orientation.x = msg.orientation.x
    # target_pose.orientation.y = msg.orientation.y
    # target_pose.orientation.z = msg.orientation.z
    # target_pose.orientation.w = msg.orientation.w

    # roll, pitch, yaw = quat2Euler(target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w)
    


    # zrot = Zrot(roll)
    # yrot = Yrot(pitch)
    # xrot = Xrot(yaw)

    # T = zrot.dot(yrot).dot(xrot)
    # global eulerZYZ
    # eulerZYZ = EulerZYZtransformer(T)
    # print('euler angle for ZYZ is', eulerZYZ)
    
    
    q_0 = target_pose.orientation.w
    q_1 = target_pose.orientation.x
    q_2 = target_pose.orientation.y
    q_3 = target_pose.orientation.z

    print("orientation q0 = ",q_0, "q1 = ",q_1, "q2 = ",q_2, "q3 = ",q_3)
    t1 = math.atan2(q_1, q_2)
    t2 = math.atan2(q_3, q_0)
    print('normalization needed?',q_0*q_0+q_1*q_1+q_2*q_2+q_3*q_3 )

    z1 = t2 - t1 
    y1 = 2*math.acos(math.sqrt(q_0*q_0 + q_3*q_3))
    z2 = t2 + t1

    # r11, r12, r21, r31, r32 = quat2rot(q_0,q_1,q_2,q_3)
    # z1, y1,z2 = rot2eulzyz(r11, r12,r21,r31,r32)
    
    
    
    global eulerZYZ
    eulerZYZ = [z1, y1, z2]
    print('the euler angles calculated are', eulerZYZ)





def pnp_cb(self,msg):
    if(self.start_flag=="0.0"):
        pass


if __name__=='__main__':
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    
    movej(Q0,50,50)
    rospy.init_node('snu_moveit_commander', anonymous=True)
    safe_offset = 300
    rospy.Subscriber('ur_pnp', String, pnp_cb, queue_size=1) #trigger
    rospy.Subscriber('cmd_moveit', Pose, cmd_moveit_cb, queue_size=1)
    rospy.Subscriber('dsr/state', RobotState, dsr_state_cb, queue_size=1)
    cmd_moveit_cb()
    cameraoffset = [-100.0, 0.0]
    if target_pose.position.x ==0 or target_pose.position.y ==0 or target_pose.position.z ==0:
        Exception
        print "[ERROR]: The Target(TF) is not Detected !!!"
    
    
    else:
        primary_goal = posx(target_pose.position.x,target_pose.position.y,target_pose.position.z+300,0,180,0)
        
        primary_goal = posx(target_pose.position.x+cameraoffset[0],target_pose.position.y+cameraoffset[1],target_pose.position.z+safe_offset,eulerZYZ[0]*RAD2DEG,eulerZYZ[1]*RAD2DEG,eulerZYZ[2]*RAD2DEG)
        print('the primary_goal is' , primary_goal) 
        print('moving towards target')
        movel(primary_goal,vel = [50,10] ,acc = [50,10])

    

    #movel(primary_goal,vel = [50,10] ,acc = [50,10])
    