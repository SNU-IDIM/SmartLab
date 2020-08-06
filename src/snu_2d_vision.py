#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
import sys
import math
import tf
import tf2_ros
import numpy as np
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist, Pose, TransformStamped
from ar_track_alvar_msgs.msg import *
from tf.transformations import *
from sensor_msgs.msg import JointState, Joy, Image

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

MM2M = 1.0 / 1000.0
M2MM  = 1000.0
###############  TO DO LIST  ################
# 1. make TF function 
# 2. organize parameter
# 3. action by flag

###############  CHANGE ##################
AR_MARKER_FRAME_PREFIX_ = 'ar_marker_'
AR_TARGET_FRAME_PREFIX_ = 'ar_target_'
CAMERA_FRAME_PREFIX_    = 'camera_link'
OBJECT_TARGET_PREFIX_   = 'object_target_'


OFFSET_FROM_TARGET_X  = 0.0   * MM2M # 보정 [mm]
OFFSET_FROM_TARGET_Y  = 0.0   * MM2M # 250.0 [mm]
OFFSET_FROM_TARGET_Z  = 175.0 * MM2M # [mm]
OFFSET_FROM_TARGET_RX = 180.0
OFFSET_FROM_TARGET_RY = 0.0
OFFSET_FROM_TARGET_RZ = 90.0




class snu_2d_vision():
    def __init__(self):
        rospy.init_node('snu_2d_vision', anonymous=True)

        self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw",Image,self.vision_cb)






        
        self.listener = tf.TransformListener()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = TransformStamped()

        ### Class Variables ###
        self.reference_frame_name  = rospy.get_param("snu_object_tracker/reference_frame",      'base_0')
        self.object_frame_name     = rospy.get_param("snu_object_tracker/object_frame",         'ar_marker_0')
        self.target_frame_name     = rospy.get_param("snu_object_tracker/target_frame",         'ar_target_0')
        self.offset_from_target_x  = rospy.get_param("snu_object_tracker/offset_from_target/x",  OFFSET_FROM_TARGET_X)
        self.offset_from_target_y  = rospy.get_param("snu_object_tracker/offset_from_target/y",  OFFSET_FROM_TARGET_Y)
        self.offset_from_target_z  = rospy.get_param("snu_object_tracker/offset_from_target/z",  OFFSET_FROM_TARGET_Z)
        self.offset_from_target_rx = rospy.get_param("snu_object_tracker/offset_from_target/rx", OFFSET_FROM_TARGET_RX)
        self.offset_from_target_ry = rospy.get_param("snu_object_tracker/offset_from_target/ry", OFFSET_FROM_TARGET_RY)
        self.offset_from_target_rz = rospy.get_param("snu_object_tracker/offset_from_target/rz", OFFSET_FROM_TARGET_RZ)
        self.tags = AlvarMarkers()
        self.cmd_pose = Pose()

        ### Topics to Subscribe ###
        rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1) # Trigger Topic
        # rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_sub_cb, queue_size=1) # AR Marker Topic
        rospy.Subscriber('objects', Float32MultiArray, self.object_sub_cb, queue_size=1) # Object Pose Topic
        
        ### Topics to Publish ###
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1) # Final target pose to be tracked


    def vision_cb(self, data):
        # pass
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.draw_image = copy.deepcopy(self.cv_image)
            self.specimen_image = copy.deepcopy(self.draw_image) # move this when you draw something in the image and change imagewindowflag
        except CvBridgeError as e:
            print(e)


    '''
        변환: (trans, rot) -> geometry_msgs/Pose
    '''
    def update_cmd_pose(self, trans, rot):
        self.cmd_pose.position.x    = trans[0] # 보정 @(arm -> 측면)
        self.cmd_pose.position.y    = trans[1] # 보정 @(arm -> 정면)
        self.cmd_pose.position.z    = trans[2]
        self.cmd_pose.orientation.x = rot[0]
        self.cmd_pose.orientation.y = rot[1]
        self.cmd_pose.orientation.z = rot[2]
        self.cmd_pose.orientation.w = rot[3]
    

    '''
        Update ROS Parameters
    '''    
    def update_ros_param(self):
        self.object_frame_name     = rospy.get_param("snu_object_tracker/object_frame")
        self.target_frame_name     = rospy.get_param("snu_object_tracker/target_frame")
        self.offset_from_target_x  = rospy.get_param("snu_object_tracker/offset_from_target/x")
        self.offset_from_target_y  = rospy.get_param("snu_object_tracker/offset_from_target/y")
        self.offset_from_target_z  = rospy.get_param("snu_object_tracker/offset_from_target/z")
        self.offset_from_target_rx = DEG2RAD * rospy.get_param("snu_object_tracker/offset_from_target/rx")
        self.offset_from_target_ry = DEG2RAD * rospy.get_param("snu_object_tracker/offset_from_target/ry")
        self.offset_from_target_rz = DEG2RAD * rospy.get_param("snu_object_tracker/offset_from_target/rz")
        

    '''
        AR_Marker -> Subscribe -> Target Pose 계산 -> TF Broadcast (새로운 Frame: TARGET_FRAME)
    '''
    def specimen_pos_check(self, msg):
        self.update_ros_param()

        #Set Region of Interest
        rowEnd=600
        colEnd=400
        rowStart=220 #224
        colStart=20

        #Offset from camera to endeffector
        cam_offsetx = 116
        cam_offsety = 43

        obj_count=1

        bgr_temp = self.specimen_image
        gray_temp=cv2.cvtColor(bgr_temp, cv2.COLOR_BGR2GRAY)

        gray=gray_temp[colStart:colEnd, rowStart:rowEnd]
        bgr=bgr_temp[colStart:colEnd, rowStart:rowEnd]

        #Canny edge detection & Hough lines transform
        edges=cv2.Canny(gray,50,200)
        cv2.imshow('Canny', edges)
        cv2.waitKey(1000)
        _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        
        for ii in range(len(contours)):
            ptAccum=np.squeeze(contours[ii])

            # FILTER1 : the specimen edge should contain more than 300 points
            if len(ptAccum) <500: 
                print('bad search : point shortage')
                continue

            x_Max=np.argmax(ptAccum[:,0]) # x maximum coordinate index
            x_Min=np.argmin(ptAccum[:,0]) # x minimum coordinate index
            y_Max=np.argmax(ptAccum[:,1]) # y maximum coordinate index
            y_Min=np.argmin(ptAccum[:,1]) # y minimum coordinate index

            #find four rectnagular Vertices using maximum coordinate
            x_Max_Vertice=[ptAccum[x_Max,0], ptAccum[x_Max,1]]
            x_Min_Vertice=[ptAccum[x_Min,0], ptAccum[x_Min,1]]
            y_Max_Vertice=[ptAccum[y_Max,0], ptAccum[y_Max,1]]
            y_Min_Vertice=[ptAccum[y_Min,0], ptAccum[y_Min,1]]

            # FILTER2
            print(ptAccum[x_Max,0], ptAccum[x_Min,0], ptAccum[y_Max,1], ptAccum[y_Min,1])            

            orientation1= float(x_Max_Vertice[1]-x_Min_Vertice[1])/float(x_Max_Vertice[0]-x_Min_Vertice[0])
            orientation2= float(y_Max_Vertice[1]-y_Min_Vertice[1])/float(y_Max_Vertice[0]-y_Min_Vertice[0])
            orientation=(orientation1+orientation2)/2.0
            theta=math.atan(orientation)

            #centroid : average of all coordinates
            centroid=[float(sum(ptAccum[:,0])/float(len(ptAccum[:,0]))), float(sum(ptAccum[:,1]))/float(len(ptAccum[:,1]))]

            #plotting for debugging 
            cv2.circle(bgr, (int(centroid[0]), int(centroid[1])),2,(0,0,255),4)
            cv2.circle(bgr, (int(y_Max_Vertice[0]), int(y_Max_Vertice[1])),1,(0,255,255),2)
            cv2.circle(bgr, (int(x_Min_Vertice[0]), int(x_Min_Vertice[1])),1,(0,255,255),2)
            cv2.circle(bgr, (int(y_Min_Vertice[0]), int(y_Min_Vertice[1])),1,(0,255,255),2)
            cv2.circle(bgr, (int(x_Max_Vertice[0]), int(x_Max_Vertice[1])),1,(0,255,255),2)
            cv2.imshow('image', bgr)
            cv2.waitKey(30)

            #Calibration 100mm / 188pixels
            px2mm_Row=(centroid[0]+rowStart-320)*100/188
            px2mm_Col=(centroid[1]+colStart-240)*100/188
            

            if centroid is not None:

                ## trnasformStamped 를 따로 def로 만들기~~ 
                self.static_transformStamped.header.stamp    = msg.header.stamp
                self.static_transformStamped.header.frame_id = CAMERA_FRAME_PREFIX_
                self.static_transformStamped.child_frame_id  = OBJECT_TARGET_PREFIX_ + str(obj_count)
                self.static_transformStamped.transform.translation.x = 0.0 - cam_offsetx + px2mm_Col
                self.static_transformStamped.transform.translation.y = 0.0 + cam_offsety - px2mm_Row
                self.static_transformStamped.transform.translation.z = 0.0
                quat = tf.transformations.quaternion_from_euler(0,0, theta*180/np.pi)
                self.static_transformStamped.transform.rotation.x = quat[0]
                self.static_transformStamped.transform.rotation.y = quat[1]
                self.static_transformStamped.transform.rotation.z = quat[2]
                self.static_transformStamped.transform.rotation.w = quat[3]
                self.broadcaster.sendTransform(self.static_transformStamped)
                obj_count = obj_count+1









        # n_tags = len(msg.markers)
        # print "Number of detected tags: %d"%n_tags
        
        # if n_tags is not 0:
        #     idx = 0
        #     for x in msg.markers:
        #         # self.static_transformStamped.header.stamp    = msg.header.stamp
        #         # self.static_transformStamped.header.frame_id = CAMERA_FRAME_PREFIX_
        #         # self.static_transformStamped.child_frame_id  = AR_MARKER_FRAME_PREFIX_ + str(msg.markers[idx].id)
        #         # self.static_transformStamped.transform.translation.x = msg.markers[idx].pose.pose.position.x
        #         # self.static_transformStamped.transform.translation.y = msg.markers[idx].pose.pose.position.y
        #         # self.static_transformStamped.transform.translation.z = msg.markers[idx].pose.pose.position.z
        #         # self.static_transformStamped.transform.rotation.x = msg.markers[idx].pose.pose.orientation.x
        #         # self.static_transformStamped.transform.rotation.y = msg.markers[idx].pose.pose.orientation.y
        #         # self.static_transformStamped.transform.rotation.z = msg.markers[idx].pose.pose.orientation.z
        #         # self.static_transformStamped.transform.rotation.w = msg.markers[idx].pose.pose.orientation.w
        #         # self.broadcaster.sendTransform(self.static_transformStamped)

        #         self.static_transformStamped.header.stamp    = msg.header.stamp
        #         self.static_transformStamped.header.frame_id = AR_MARKER_FRAME_PREFIX_ + str(msg.markers[idx].id)
        #         self.static_transformStamped.child_frame_id  = AR_TARGET_FRAME_PREFIX_ + str(msg.markers[idx].id)
        #         self.static_transformStamped.transform.translation.x = 0.0 + self.offset_from_target_x
        #         self.static_transformStamped.transform.translation.y = 0.0 + self.offset_from_target_y
        #         self.static_transformStamped.transform.translation.z = 0.0 + self.offset_from_target_z
        #         quat = tf.transformations.quaternion_from_euler(self.offset_from_target_rx, self.offset_from_target_ry, self.offset_from_target_rz)
        #         self.static_transformStamped.transform.rotation.x = quat[0]
        #         self.static_transformStamped.transform.rotation.y = quat[1]
        #         self.static_transformStamped.transform.rotation.z = quat[2]
        #         self.static_transformStamped.transform.rotation.w = quat[3]
        #         self.broadcaster.sendTransform(self.static_transformStamped)

        #         idx += 1
        #     return 1
        # else:
        #     return -1


    def object_sub_cb(self, msg):
        self.update_ros_param()
        self.static_transformStamped.header.stamp    = rospy.Time.now()
        self.static_transformStamped.header.frame_id = self.object_frame_name
        self.static_transformStamped.child_frame_id  = self.target_frame_name
        
        self.static_transformStamped.transform.translation.x = self.offset_from_target
        self.static_transformStamped.transform.translation.y = 0.0
        self.static_transformStamped.transform.translation.z = 0.0
        
        quat = tf.transformations.quaternion_from_euler(0.0, -math.pi/2.0, 0.0)
        self.static_transformStamped.transform.rotation.x = quat[0]
        self.static_transformStamped.transform.rotation.y = quat[1]
        self.static_transformStamped.transform.rotation.z = quat[2]
        self.static_transformStamped.transform.rotation.w = quat[3]
        #print "%s"%self.static_transformStamped
        
        self.broadcaster.sendTransform(self.static_transformStamped)

    '''
        "/ur_pnp" Subscribe -> if) data == "search" -> TF Subscribe -> "cmd_pose" Publish
    '''
    def pnp_cb(self, msg): # Version 2: "/tf" topic 이용
        print "Subscribed '/ur_pnp' -> '%s'"%msg.data
        if msg.data == 'search':
            # Update the Target Frame Name (임시로 rosparam 이용해 target_frame_name 수정 - SMACH 구성 후 수정 필요)
            self.target_frame_name = rospy.get_param("snu_object_tracker/target_frame")
            try:
                print "Trying to search the target: %s ..."%self.object_frame_name
                self.listener.waitForTransform(self.reference_frame_name, self.target_frame_name, rospy.Time(), rospy.Duration(0.5))
                (trans,rot) = self.listener.lookupTransform(self.reference_frame_name, self.target_frame_name, rospy.Time(0))

                self.update_cmd_pose(trans, rot)
                self.pub1.publish(self.cmd_pose)
                print(self.cmd_pose)

            except (tf.Exception):
                print "[ERROR]: The Target(TF) is not Detected !!!"
                pass
            return 1
        else:
            return -1

    
if __name__ == "__main__":
    twod_vision = snu_2d_vision()

    #test = String()
    #test.data = '1'
    #ar.pnp_cb(test)

    while not rospy.is_shutdown():
        #ar.pnp_cb(test)
        pass

    print 'good bye!'
