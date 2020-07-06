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
import copy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
from distutils.version import LooseVersion

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


Q0 = [8.031315803527832, 27.553855895996094, -110.20037841796875, 1.7169572114944458, -96.97026062011719, 10.950394630432129]

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
        self.etarget_pose = Pose()
        self.drl_pose = Q0
        self.eulerZYZ = np.zeros(3)
        self.eeulerZYZ = np.zeros(3)
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
        rospy.sleep(2)
        self.pnp_pub.publish("open")
        rospy.sleep(1)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw",Image,self.vision_cb)
        self.imagewindowflag = 0

    def vision_cb(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.imagewindowflag ==0:
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.imshow('image', self.cv_image)
            cv2.waitKey(1)
        

    def update_cmd_pose(self, trans, rot):
        self.target_pose.position.x    = M2MM(trans[0])# + self.offset_x) # 보정 @(arm -> 측면)
        self.target_pose.position.y    = M2MM(trans[1])# + self.offset_y) # 보정 @(arm -> 정면)
        self.target_pose.position.z    = M2MM(trans[2])# + self.offset_z)
        self.target_pose.orientation.x = rot[0]
        self.target_pose.orientation.y = rot[1]
        self.target_pose.orientation.z = rot[2]
        self.target_pose.orientation.w = rot[3]
        print(self.target_pose)
    
    def eupdate_cmd_pose(self, etrans, erot):
        print ('eupdate_cmd_pose')
        self.etarget_pose.position.x    = M2MM(etrans[0])# + self.offset_x) # 보정 @(arm -> 측면)
        self.etarget_pose.position.y    = M2MM(etrans[1])# + self.offset_y) # 보정 @(arm -> 정면)
        self.etarget_pose.position.z    = M2MM(etrans[2])# + self.offset_z)
        self.etarget_pose.orientation.x = erot[0]
        self.etarget_pose.orientation.y = erot[1]
        self.etarget_pose.orientation.z = erot[2]
        self.etarget_pose.orientation.w = erot[3]
        

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

    def eupdateEulZYZ(self):
        eq_w = self.etarget_pose.orientation.w
        eq_x = self.etarget_pose.orientation.x
        eq_y = self.etarget_pose.orientation.y
        eq_z = self.etarget_pose.orientation.z

        t1 = math.atan2(eq_x, eq_y)
        t2 = math.atan2(eq_z, eq_w)

        z1 = t2 - t1 
        y1 = 2*math.acos(math.sqrt(eq_w*eq_w + eq_z*eq_z))
        z2 = t2 + t1  

        self.eeulerZYZ = [RAD2DEG(z1), RAD2DEG(y1), RAD2DEG(z2)]
        print('The Euler angles are calculated:', self.eulerZYZ)


    def search_target(self,target_frame_name = 'ar_target_2'):
        print "CMDDDDDDDDDDDDDDDDD"
        
        reference_frame_name = 'base_0'
        end_effector_frame = 'link6'
        camera_frame = 'camera_link'
        
        listener = tf.TransformListener()
        

        try:
            print "Trying to search the target: %s ..."%target_frame_name
            self.listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(1.0))
            self.listener.waitForTransform(end_effector_frame, target_frame_name, rospy.Time(), rospy.Duration(1.0))
            
            (trans,rot) = self.listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
            (etrans,erot) = self.listener.lookupTransform(end_effector_frame, target_frame_name, rospy.Time(0))
            
            self.update_cmd_pose(trans, rot)
            self.eupdate_cmd_pose(etrans,erot)

            self.updateEulZYZ()
            self.eupdateEulZYZ()
            
            self.drl_pose = deepcopy(posx(self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z ,self.eulerZYZ[0], self.eulerZYZ[1], self.eulerZYZ[2]))
            self.edrl_pose = deepcopy(posx(self.etarget_pose.position.x, self.etarget_pose.position.y, self.etarget_pose.position.z ,self.eeulerZYZ[0], self.eeulerZYZ[1], self.eeulerZYZ[2]))
            
            print('Target DRL Pose from base coordinates: ' , self.drl_pose)
            print('Target EDRL Pose from end_effector: ' , self.edrl_pose)
            
        except (Exception):
            print "[ERROR]: The Target(TF) is not Detected !!!"
            print('the drlpose is', self.drl_pose)
            self.drl_pose[0] = 0
            pass

    def UpdateParam(self, dx, dy, dz):
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/x', dx)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/y', dy)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/z', dz)
        rospy.sleep(2)
    
    def movel_z(self, distance): # distance [m]
        movel(posx(0, 0, M2MM(distance), 0, 0, 0), vel=[50,10], acc=[50,10], ref=DR_TOOL, mod=DR_MV_MOD_REL)

    def window_drawing(self):
        red = (0, 0, 255)
        green = (0, 255, 0)
        blue = (255, 0, 0)
        white = (255, 255, 255)
        yellow = (0, 255, 255)
        center_x = int(np.size(self.cv_image,1) / 2.0)
        center_y = int(np.size(self.cv_image,0) / 2.0)
        thickness = 2
        # print('centerx is', center_x )

        location = (center_x - 200, center_y - 100)
        font = cv2.FONT_ITALIC
        fontScale = 3.5
        # cv2.putText(self.cv_image, 'OpenCV', location, font, fontScale, yellow, thickness)


    def pnp_cb(self, msg):
        self.start_flag = msg.data
        if(self.start_flag=="approach"):
            Q0 = [-35.86109161376953, -12.6979341506958, -123.48367309570312, -118.48633575439453, -64.00657653808594, 142.61135864257812]
            
            Q0 = [-14.99778938293457, -23.47661781311035, -125.91959381103516, -104.70079040527344, -83.42718505859375, 150.46311950683594]
            
            movej(Q0, 50, 50) # Search pose
            self.UpdateParam(0.0, -0.12, 0.25)
            rospy.sleep(1)

            self.search_target()
            if not self.drl_pose[0] ==0:
                
                
                ########searching for the ar tag#2 which is placed on 3dp door####
                #Searching is consisted of three steps
                #TF: ar_target  -- robot base
                # ar target which is a TF made from ar_marker it is facing the marker tf(z axis is towards each other)
                #consist of three move which is  conducting scan, move and feedback
                movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 1st approach
                self.search_target()
                movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 2nd approach
                self.search_target()
                movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 2nd approach
                self.search_target()

            #three movel functions are integrated as movesx
                #define three coordinate points reference:1 mode:1
                self.openway   = posx(0,0,0,0,0,0)
                self.openstart = posx(0,0,0,0,0,0)
                self.openend = posx(0,0,0,0,0,0)
                
                #moving side ways to get to the right side of the handle
                self.openway[0] = 50
                self.openway[1] = 150
                
                #going in z direction(from end effector tf) to pull
                self.openstart[2]= 80
                #end point of the pulling motion
                self.openend[1] = -300
                self.openend[2] = -300

                self.movelist = [self.openway, self.openstart,self.openend]
                movesx(self.movelist,vel=[100,50], acc=[100,50],ref = 1,mod =1)
        ################DOOR CLOSING EXECUTED STARTING PICKING UP PLATE#################


                #move inside to see the ar tag #1 on the 3dp printing plate
                movel(posx(-100,-100,250,0,-30,0), vel=[100,50], acc=[100,50],ref = 1, mod = 1) # 2nd approach
                
                
            #Search for the ar marker_1 this time scanning was used several times and also tf is different
            #the TF used is ar_marker_1 and endeffector tf
            #Also manual tilting motion was given to see the ar tag
                #First approach
                self.search_target('ar_marker_1')
                pick_start = posx(0,0,0,0,-10,0)
                pick_start[0] = self.edrl_pose[0] 
                pick_start[1] = self.edrl_pose[1]

                movel(pick_start,vel=[30,30], acc=[30,30],ref = 1,mod =1)
                
                
                #Second approach
                self.search_target('ar_marker_1')
                pick_start = posx(0,0,0,0,-20,0)
                pick_start[0] = self.edrl_pose[0]
                pick_start[1] = self.edrl_pose[1]

                movel(pick_start,vel=[30,30], acc=[30,30],ref = 1,mod =1)

                
                #for dragging go inside
                movel(posx(-50,0,0,0,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)
                
                #Third search and execution
                self.search_target('ar_marker_1')
                pick_start = posx(0,0,0,0,60,0)
                pick_start[0] = self.edrl_pose[0]
                pick_start[1] = self.edrl_pose[1]
                movel(pick_start,vel=[30,30], acc=[30,30],ref = 1,mod =1)
                
                
            # move in a given sequence so that it can pick up the plate
                #########ERASE AFTER DEBUGGING#############    
                # movel(posx(-178,20,0,0,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)
                # movel(posx(-6,0,-50,0,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)
                # movel(posx(30,0,0,0,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)
                # movel(posx(0,0,0,90,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)
                #move out
                #movel(posx(0,0,-70,0,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)
                #move down
                #movel(posx(0,55.5,0,0,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)
                #going in
                #movel(posx(0,0,70,0,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)
                #########ERASE AFTER DEBUGGING#############    
                
                #going down motion for dragging
                t1 = posx(-178,20,0,0,0,0)
                #dragging which is out of the tray
                t2 = posx(-6,0,-50,0,0,0)
                #lift up after dragging
                t3 = posx(30,0,0,0,0,0)
                #rotate for picking up
                t4 = posx(0,0,0,90,0,0)

                ############################Rotation made###Tf changed so please consider tf change##

                #move out
                t5 = posx(0,0,-70,0,0,0)
                #move down
                t6 = posx(0,55.5,0,0,0,0)
                #going in
                t7 = posx(0,0,70,0,0,0)
                
                
                
                drag = [t1,t2,t3,t4,t5,t6,t7]
                movesx(drag,vel=[30,30], acc=[30,30],ref = 1,mod =1)

                #close gripper
                self.pnp_pub.publish("close")

                rospy.sleep(1)
                #going out
                transport1 = posx(0,0,-100,0,0,0)
                
                #going up
                transport2 = posx(0,-100,0,0,0,0)
               
                #moving towards plate
                transport3 = posx(0,0,-700,0,0,0)

                transport = [transport1, transport2,transport3]

                movesx(transport, vel=[30,30], acc=[30,30],ref = 1,mod =1)

            #########ERASE AFTER DEBUGGING#############    
                #movel(posx(0,0,-100,0,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)
                
                #going up
                #movel(posx(0,-100,0,0,0,0),vel=[30,30], acc=[30,30],ref = 1,mod =1)

                #moving towards plate

                #movel(posx(0,0,-700,0,0,0),vel=[100,30], acc=[30,30],ref = 1,mod =1)
                # movel(posx(250,0,0,0,0,0),vel=[100,30], acc=[30,30],ref = 1,mod =1)
            #########ERASE AFTER DEBUGGING#############    
                
######################Robot picked the plate###########################
##Now approach towards the cnc machine and place the 3dp plate on universal jig

                ## rotate the plate in the same orientation so that the plate does not fall
                temp0 = posx(0,0,0,90,0,0)
                temp1 = posx(0.0, 0.0, 0.0, 0.0, -90.0, 0.0)
                temp_path = [temp0, temp1, temp1]
                movesx(temp_path,vel=[30,30], acc=[30,30],ref = 0,mod = 1)
        
        
        
        
            #Search for ar marker that is on cnc, the marker number is 0
                self.search_target('ar_marker_0')
                self.UpdateParam(0, 0, 0)


########################################################################################
####################AMR MOVING CODE FLAG NEEDS TO BE PLACED HERE#########################
###########################################################################
##First Approach towards cnc
                drop_start = posx(0,0,0,0,0,0)
                drop_start[0] = self.edrl_pose[0] 
                drop_start[1] = self.edrl_pose[1]
                drop_start[2] = self.edrl_pose[2] - 350
                movel(drop_start,vel=[30,30], acc=[30,30],ref = 1,mod =1)
                print('First approach done')


##Second Approach towards cnc
                self.search_target('ar_marker_0')
                drop_way = posx(0,0,0,0,0,0)
                drop_way[0] = self.edrl_pose[0] 
                drop_way[1] = self.edrl_pose[1]
                drop_way[2] = self.edrl_pose[2] - 350
                movel(drop_way,vel=[170,130], acc=[70,30],ref = 1,mod =1)
                print('Second approach done')

###Going up motion
                drop_way1 = posx(0,0,0,0,0,0)
                drop_way1[1] = -170
##Going in motion
                drop_way2 = posx(0,0,0,0,0,0)
                drop_way2[2] = 270
                drop_way_integrated = [drop_way1, drop_way2]
                
                
                movesx(drop_way_integrated,vel=[70,150], acc=[70,30],ref = 1,mod =1)

###Open finger
                self.pnp_pub.publish("open")


if __name__=='__main__':
    DRLInterface()
    
    while not rospy.is_shutdown():
        pass
    
