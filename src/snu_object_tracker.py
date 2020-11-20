#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, threading, time, math
import numpy as np

import rospy
import tf, tf2_ros
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist, Pose, TransformStamped
from ar_track_alvar_msgs.msg import *
from tf.transformations import *

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

MM2M = 1.0 / 1000.0
M2MM  = 1000.0

AR_MARKER_FRAME_PREFIX_ = 'ar_marker_'
AR_TEMP_FRAME_PREFIX_  = 'ar_temp_'
AR_CALIB_FRAME_PREFIX_  = 'ar_calib_'
AR_TARGET_FRAME_PREFIX_ = 'ar_target_'
CAMERA_FRAME_PREFIX_    = 'camera_link'

OFFSET_FROM_TARGET_X  = 0.0   * MM2M # 보정 [mm]
OFFSET_FROM_TARGET_Y  = 0.0   * MM2M # 250.0 [mm]
OFFSET_FROM_TARGET_Z  = 175.0 * MM2M # [mm]
OFFSET_FROM_TARGET_RX = 180.0
OFFSET_FROM_TARGET_RY = 0.0
OFFSET_FROM_TARGET_RZ = -90.0


class snu_object_tracker():
    def __init__(self):
        rospy.init_node('snu_obejct_tracker', anonymous=True)
        
        ## ROS Topic - publish & subscribe
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1) # Final target pose to be tracked
        rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1) # Trigger topic
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_sub_cb, queue_size=1) # AR Marker topic
        rospy.Subscriber('objects', Float32MultiArray, self.object_sub_cb, queue_size=1) # Object pose topic

        ## ROS TF related
        self.listener = tf.TransformListener()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = TransformStamped()

        ## Class variables (translation, rotation, ...)
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


    def update_cmd_pose(self, trans, rot):
        '''
            Descriptions:
                1. Update 'self.cmd_pose' with trans, rot from TF
                2. 'self.cmd_pose' is going to be published with 'cmd_moveit' topic
        '''
        self.cmd_pose.position.x    = trans[0] # 보정 @(arm -> 측면)
        self.cmd_pose.position.y    = trans[1] # 보정 @(arm -> 정면)
        self.cmd_pose.position.z    = trans[2]
        self.cmd_pose.orientation.x = rot[0]
        self.cmd_pose.orientation.y = rot[1]
        self.cmd_pose.orientation.z = rot[2]
        self.cmd_pose.orientation.w = rot[3]
    

    def update_ros_param(self):
        '''
            Descriptions:
                1. Update target offset from ar_marker with new ROS parameters
                2. These ROS parameters will be updated in another ROS node (snu_drl_interface)
        '''    
        self.object_frame_name     = rospy.get_param("snu_object_tracker/object_frame")
        self.target_frame_name     = rospy.get_param("snu_object_tracker/target_frame")
        self.offset_from_target_x  = rospy.get_param("snu_object_tracker/offset_from_target/x")
        self.offset_from_target_y  = rospy.get_param("snu_object_tracker/offset_from_target/y")
        self.offset_from_target_z  = rospy.get_param("snu_object_tracker/offset_from_target/z")
        self.offset_from_target_rx = DEG2RAD * rospy.get_param("snu_object_tracker/offset_from_target/rx")
        self.offset_from_target_ry = DEG2RAD * rospy.get_param("snu_object_tracker/offset_from_target/ry")
        self.offset_from_target_rz = DEG2RAD * rospy.get_param("snu_object_tracker/offset_from_target/rz")
        

    def ar_sub_cb(self, msg):
        '''
            Descriptions:
                1. Callback function of subscribing 'ar_pose_marker'
                2. Filter the flippy AR Marker orientation (TF: 'link6' -> 'ar_calib_[]')
                3. Broadcast a new target pose frame (TF: 'ar_calib_[]' -> 'ar_target_[]')
        '''
        self.update_ros_param()

        n_tags = len(msg.markers)
        # print("[DEBUG] Number of detected tags: {}".format(n_tags))
        
        if n_tags is not 0:
            idx = 0
            for x in msg.markers:
                try:
                    ## Some of the frames
                    ar_marker_frame = AR_MARKER_FRAME_PREFIX_ + str(msg.markers[idx].id)
                    ar_temp_frame   = AR_TEMP_FRAME_PREFIX_   + str(msg.markers[idx].id)
                    ar_calib_frame  = AR_CALIB_FRAME_PREFIX_  + str(msg.markers[idx].id)
                    base_frame      = 'base_0'
                    reference_frame = 'link6'
                    
                    ## Some of the constants for filter
                    EPSILON = 0.05         ## sin // cos 비교
                    TOLERANCE_ANGLE = 5    ## angle [deg] 비교
                    DISTANCE_FILTER = 1.0  ## if) link6 - ar_marker 사이 거리 < DISTANCE -> filter 적용

                    ## Look up TF of "base_0" - "link6" for further calculation
                    self.listener.waitForTransform(base_frame, reference_frame, rospy.Time(), rospy.Duration(1.0))
                    (trans_base2ref,rot_base2ref) = self.listener.lookupTransform(base_frame, reference_frame, rospy.Time(0))

                    ## Look up TF of "link6" - "ar_marker_[]" for further calculation
                    self.listener.waitForTransform(reference_frame, ar_marker_frame, rospy.Time(), rospy.Duration(1.0))
                    (trans_ref2ar,rot_ref2ar) = self.listener.lookupTransform(reference_frame, ar_marker_frame, rospy.Time(0))

                    ## Calculate distance, yaw_angle between 'link6' - 'ar_marker_[]' to decide whether to apply the filter below
                    dist = math.sqrt(trans_ref2ar[0]*trans_ref2ar[0] + trans_ref2ar[1]*trans_ref2ar[1] + trans_ref2ar[2]*trans_ref2ar[2])
                    yaw = euler_from_quaternion(rot_ref2ar)[2]
                    sign = int(np.sign(math.sin(yaw)))

                    ## Filter implementation (range: 500 mm from AR_marker)
                    if dist < DISTANCE_FILTER:

                        ## 0 deg (not flipped)
                        if abs(RAD2DEG*yaw) > 180.0-TOLERANCE_ANGLE: 
                            print("[Debug] 0 deg")
                            q_rot_calib = quaternion_from_euler(0, 0, 0)

                        ## 180 deg (flipped), compensation: +180 deg
                        elif abs(RAD2DEG*yaw) < TOLERANCE_ANGLE :  
                            print("[Debug] 180 deg")
                            q_rot_calib = quaternion_from_euler(0, 0, math.pi)

                        ## 90 deg (flipped), compensation: -90 deg
                        elif sign == 1 and abs(math.sin(yaw)) > 1.0 - EPSILON:
                            print("[Debug] 90 deg")
                            q_rot_calib = quaternion_from_euler(0, 0, -math.pi/2.0)
                        ## -90 deg (flipped), compensation: +90 deg
                        elif sign == -1 and abs(math.sin(yaw)) > 1.0 - EPSILON:
                            print("[Debug] -90 deg")
                            q_rot_calib = quaternion_from_euler(0, 0, math.pi/2.0)

                        else:
                            q_rot_calib = quaternion_from_euler(0, 0, 0)
                            print('[Original1]: {}'.format(RAD2DEG * np.asarray(euler_from_quaternion(q_rot_calib))))

                    else:
                        q_rot_calib = quaternion_from_euler(0, 0, 0)
                        print('[Debug] Distance: {}'.format(dist))

                    ## Create a new frame 'ar_calib_[]' w.r.t. 'link6' frame (filtered ar_marker_pose)
                    self.static_transformStamped.header.stamp    = msg.header.stamp
                    self.static_transformStamped.header.frame_id = reference_frame
                    self.static_transformStamped.child_frame_id  = AR_CALIB_FRAME_PREFIX_  + str(msg.markers[idx].id)
                    self.static_transformStamped.transform.translation.x = 0.0 + trans_ref2ar[0] 
                    self.static_transformStamped.transform.translation.y = 0.0 + trans_ref2ar[1]
                    self.static_transformStamped.transform.translation.z = 0.0 + trans_ref2ar[2]
                    q_rot_calib = tf.transformations.quaternion_multiply(rot_ref2ar, q_rot_calib)
                    self.static_transformStamped.transform.rotation.x = q_rot_calib[0]
                    self.static_transformStamped.transform.rotation.y = q_rot_calib[1]
                    self.static_transformStamped.transform.rotation.z = q_rot_calib[2]
                    self.static_transformStamped.transform.rotation.w = q_rot_calib[3]
                    self.broadcaster.sendTransform(self.static_transformStamped)

                    ## Create a new frame 'ar_target_[]' w.r.t. 'ar_calib_[]' (this coordinate will be used as a target pose)
                    self.static_transformStamped.header.stamp    = msg.header.stamp
                    self.static_transformStamped.header.frame_id = AR_CALIB_FRAME_PREFIX_  + str(msg.markers[idx].id)
                    self.static_transformStamped.child_frame_id  = AR_TARGET_FRAME_PREFIX_ + str(msg.markers[idx].id)
                    self.static_transformStamped.transform.translation.x = 0.0 + self.offset_from_target_x
                    self.static_transformStamped.transform.translation.y = 0.0 + self.offset_from_target_y
                    self.static_transformStamped.transform.translation.z = 0.0 + self.offset_from_target_z
                    q_target = tf.transformations.quaternion_from_euler(self.offset_from_target_rx, self.offset_from_target_ry, self.offset_from_target_rz)
                    self.static_transformStamped.transform.rotation.x = q_target[0]
                    self.static_transformStamped.transform.rotation.y = q_target[1]
                    self.static_transformStamped.transform.rotation.z = q_target[2]
                    self.static_transformStamped.transform.rotation.w = q_target[3]
                    self.broadcaster.sendTransform(self.static_transformStamped)

                except:
                    print("[ERROR] TF error with AR-Marker")

                idx += 1
            return 1
        else:
            return -1


    def object_sub_cb(self, msg):
        '''
            Descriptions:
                1. 3D object detection package related stuff
                2. Currently, we are not using this code (이전 버전)
        '''
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
        
        self.broadcaster.sendTransform(self.static_transformStamped)


    def pnp_cb(self, msg): # Version 2: "/tf" topic 이용
        '''
            Descriptions:
                1. Callback function of subscribing 'ur_pnp' topic
                2. Calculate the difference in pose between 'link6' - 'ar_target_[]' and publish 'dsr_moveit' topic
                3. Currently, we are not using this code (이전 버전)
        '''
        if msg.data == 'search':
            # Update the Target Frame Name (임시로 rosparam 이용해 target_frame_name 수정 - SMACH 구성 후 수정 필요)
            self.target_frame_name = rospy.get_param("snu_object_tracker/target_frame")
            try:
                print("[Debug] Trying to search the target: ".format(self.object_frame_name))
                self.listener.waitForTransform(self.reference_frame_name, self.target_frame_name, rospy.Time(), rospy.Duration(0.5))
                (trans,rot) = self.listener.lookupTransform(self.reference_frame_name, self.target_frame_name, rospy.Time(0))

                self.update_cmd_pose(trans, rot)
                self.pub1.publish(self.cmd_pose)

            except (tf.Exception):
                print("[ERROR]: The Target(TF) is not Detected !!!")
                pass
            return 1
        else:
            return -1

    
if __name__ == "__main__":
    ObjectTracker = snu_object_tracker()

    #test = String()
    #test.data = '1'
    #ar.pnp_cb(test)

    while not rospy.is_shutdown():
        #ar.pnp_cb(test)
        pass

    print 'good bye!'