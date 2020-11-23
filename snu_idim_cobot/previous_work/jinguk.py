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
        self.dsr_flag = 'idle'

        self.target_pose = Pose()
        self.etarget_pose = Pose()
        self.drl_pose = Q0
        self.eulerZYZ = np.zeros(3)
        self.eeulerZYZ = np.zeros(3)
        self.start_flag = '0.0'

        self.offset_x = 0.0
        self.offset_y = -0.12
        self.offset_z = 0.2

        self.gripperctrl = rospy.ServiceProxy("/R_001/dsr" + '/io/set_digital_output'     , SetCtrlBoxDigitalOutput)
        

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        rospy.sleep(1)
        # movej(Q0, 50, 50)

        rospy.init_node('snu_moveit_commander', anonymous=True)

        self.listener = tf.TransformListener()
        rospy.Subscriber('/R_001/ur_status', URStatus, self.dsr_status_cb, queue_size=1) #trigger
        rospy.Subscriber('/R_001/ur_pnp', String, self.pnp_cb, queue_size=1) #trigger
        self.pnp_pub = rospy.Publisher('/R_001/ur_pnp', String, queue_size=1)
        self.status_pub = rospy.Publisher('/R_001/ur_status', URStatus, queue_size=1)
        rospy.sleep(2)
        # self.pnp_pub.publish("open")
        self.gripper_open()
        rospy.sleep(1)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw",Image,self.vision_cb)
        self.imagewindowflag = 0
    
    def dsr_status_cb(self, data):
        self.dsr_flag = data.status

    def gripper_close(self):
        self.gripperctrl(14,0)
        rospy.sleep(1)
        self.gripperctrl(13,1)

    def gripper_open(self):
        self.gripperctrl(13,0)
        rospy.sleep(1)
        self.gripperctrl(14,1)
        
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


    def search_target(self,target_frame_name = 'ar_target_0'):
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
            # continue
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
        if(self.start_flag=="25"):
            self.status_pub.publish(URStatus(status='working'))
            Q0 = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
            
            movej(Q0, 50, 50) # Search pose
            self.UpdateParam(0.0, -0.12, 0.25)
            rospy.sleep(1)

            self.search_target()
            print(self.drl_pose)
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

                ###############
                ##여기서부터 self.drl_pose 변수 이용할 것 여기서부터 base 0 frame 기준 받은 것 (orientation 은 그대로 유지해고 position만 사용해도됨)
                # self.drl_pose
            #three movel functions are integrated as movesx
                #define three coordinate points reference:1 mode:1
                # f = open("jk.drl",'w')

                traject = traj_gen(self.drl_pose)
                print("/////////////////////////////////////////////")
                print(traject)
                print(np.size(traject),0)
                print(traject[0][1])
                # set_velx([100,50])
                # set_accx([100,50])

                # movel(posx(traject[0][0],traject[0][1],500 ,153.61219787597656, -178.1796417236328, 155.2996826171875))
                for i in range(np.size(traject,0)):
                    movel(posx(traject[i][0]+100,traject[i][1], 75 ,0, 180,0),vel=[100,50], acc=[100,50])
                
                #     movel(traject[i][0],traject[i][1],150,153.61219787597656, -178.1796417236328, 155.2996826171875)
                # # data = 'movesx(' +str(traject)+')'+'\n'
                # set_stiff = 'set_stiffness([])'
                # f.write(data)
                # f.close()
                # f = open('jk1.drl','w')
                # pose = drl.drl_pose
                # traject = traj_gen(pose)

                # print(traject) 
                # # f.write("gi\n ")
                # set_velx = 'set_velx([100,50])'+'\n'
                # set_accx = 'set_accx([100,50])'+'\n'

                # set_velj = 'set_velj(30)'+'\n'
                # set_accj = 'set_accj(30)'+'\n'

                # data = 'movesx(' + str(traject)+')'+'\n'
                # set_stiff = 'set_stiffness([])'
                
                
                # f.write(set_velx)
                # f.write(set_accx)
                # f.write(set_velj)
                # f.write(set_accj)
                # # f.write(data)
                
                # f.write("stx_print   = [3000,3000,0.01,200,200,200]")
                # f.write("set_stiffnessx(stx_print)")
                # f.write("task_compliance_ctrl(stx_print)")
                # f.close()                
            
                
def traj_gen(position):
    radius = 150
    gripper_size = 50
    r_step = np.round(radius/gripper_size)
    path = []
    path.append([position[0],position[1],position[2], 0, 180, 0])
    if(r_step <= 1):
        path.append([position[0],position[1]+50,position[2], 0, 180, 0])
        path.append([position[0]+50,position[1],position[2], 0, 180, 0])
        path.append([position[0],position[1]-50,position[2], 0, 180, 0])
        path.append([position[0]-50,position[1],position[2], 0, 180, 0])
    else:
        dev = 16
        for i in range(r_step):
            for j in range(dev):
                path.append([position[0]+np.sin(2*np.pi*j/dev)*gripper_size*(2*(i+(j+0.0)/dev)+1)/2,
                                        position[1]+np.cos(2*np.pi*j/dev)*gripper_size*(2*(i+(j+0.0)/dev)+1)/2,
                                        position[2], 0, 180, 0])
    return path


if __name__=='__main__':
    drl = DRLInterface()


    while not rospy.is_shutdown():
        pass
    
