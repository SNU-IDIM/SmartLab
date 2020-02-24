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
import cv2 as cv2


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



EPSILON = 0.0000001

def DEG2RAD(degree):
    return (math.pi/180.0)*degree

def RAD2DEG(radian):
    return (180.0/math.pi)*radian

def M2MM(meter):
    return 1000.0*meter

def MM2M(milimeter):
    return (1.0/1000.0)*milimeter


# Q0 = [0.0,    0.0,    -90.0,    0.0,    -90.0,    0.0]
Q0 = [8.031315803527832, 27.553855895996094, -110.20037841796875, 1.7169572114944458, -96.97026062011719, 10.950394630432129]
Q_SEARCH_RIGHT = [-1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_LEFT  = [1.587211119647868, 0.04192579550122713, -2.42067574545383, 0.02488730488522477, 0.060036456046744055, 5.683802467473106e-05]
Q_SEARCH_FRONT = [0.006254249687429258, 0.0706465647310261, -1.8816342308005074, -0.009305934771632234, -0.518931153024292, 0.012760136888951999]

#Q_SEARCH_RIGHT = [-0.021912608043065707, 0.3745233068645807, -2.515318099008636, -0.0016689710660107685, -0.9671584417422292, 0.00014467861565142695]

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


class DRLInterface():
    def __init__(self):
        self.target_pose = Pose()
        self.drl_pose = Q0
        self.eulerZYZ = np.zeros(3)
        self.start_flag = '0.0'

        self.offset_x = 0.0
        self.offset_y = -0.12
        self.offset_z = 0.2

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        rospy.sleep(1)
        movej(Q0, 50, 50)

        rospy.init_node('snu_moveit_commander', anonymous=True)

        self.listener = tf.TransformListener()
        rospy.Subscriber('/R_001/ur_pnp', String, self.pnp_cb, queue_size=1) #trigger
        self.pnp_pub = rospy.Publisher('/R_001/ur_pnp', String, queue_size=1)

    def update_cmd_pose(self, trans, rot):
        self.target_pose.position.x    = M2MM(trans[0])# + self.offset_x) # 보정 @(arm -> 측면)
        self.target_pose.position.y    = M2MM(trans[1])# + self.offset_y) # 보정 @(arm -> 정면)
        self.target_pose.position.z    = M2MM(trans[2])# + self.offset_z)
        self.target_pose.orientation.x = rot[0]
        self.target_pose.orientation.y = rot[1]
        self.target_pose.orientation.z = rot[2]
        self.target_pose.orientation.w = rot[3]
        print(self.target_pose)


    def updateEulZYZ(self):
        q_w = self.target_pose.orientation.w
        q_x = self.target_pose.orientation.x
        q_y = self.target_pose.orientation.y
        q_z = self.target_pose.orientation.z

        t1 = math.atan2(q_x, q_y)
        t2 = math.atan2(q_z, q_w)

        z1 = t2 - t1 
        y1 = 2*math.acos(math.sqrt(q_w*q_w + q_z*q_z))
        z2 = t2 + t1  

        self.eulerZYZ = [RAD2DEG(z1), RAD2DEG(y1), RAD2DEG(z2)]
        print('The Euler angles are calculated:', self.eulerZYZ)


    def search_target(self):
        print "CMDDDDDDDDDDDDDDDDD"
        target_frame_name = 'ar_target_1'
        reference_frame_name = 'base_0'
        
        listener = tf.TransformListener()

        try:
            print "Trying to search the target: %s ..."%target_frame_name
            self.listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(1.0))
            (trans,rot) = self.listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
            self.update_cmd_pose(trans, rot)
            self.updateEulZYZ()
            self.drl_pose = deepcopy(posx(self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z ,self.eulerZYZ[0], self.eulerZYZ[1], self.eulerZYZ[2]))
            print('Target DRL Pose: ' , self.drl_pose)

        except (Exception):
            print "[ERROR]: The Target(TF) is not Detected !!!"
            pass

    def UpdateParam(self, dx, dy, dz):
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/x', dx)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/y', dy)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/z', dz)
        rospy.sleep(2)
    
    def movel_z(self, distance): # distance [m]
        movel(posx(0, 0, M2MM(distance), 0, 0, 0), vel=[50,10], acc=[50,10], ref=DR_TOOL, mod=DR_MV_MOD_REL)


    def pnp_cb(self, msg):
        self.start_flag = msg.data

        movej(Q0, 50, 50)
        
        if(self.start_flag=="approach"):
            #self.pnp_pub.publish("open")

            self.UpdateParam(0.0, -0.12, 0.2)
            self.search_target()
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 1st approach

            self.search_target()
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 2nd approach

            self.UpdateParam(0.0, 0.0, 0.2)
            self.search_target()
            movel(self.drl_pose, vel=[100,30], acc=[100,30]) # alignment

            self.movel_z(0.040)
            #self.pnp_pub.publish("close")
            self.movel_z(-0.040)

            #move_z = posx(0, 0, M2MM(0.040), 0, 0, 0)
            #movel(move_z, vel=[50,10], acc=[50,10], ref=DR_TOOL, mod=DR_MV_MOD_REL)
            

        
        if(self.start_flag=="engage"):
            self.UpdateParam_engage()
            self.search_target()

            movel(self.drl_pose, vel=[50,10], acc=[50,10])


if __name__=='__main__':
    DRLInterface()
    
    while not rospy.is_shutdown():
        pass
    