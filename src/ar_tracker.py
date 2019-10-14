#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
import sys
import math
import tf
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, TransformStamped
from ar_track_alvar_msgs.msg import *
from tf.transformations import *

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

MM2M = 1.0 / 1000.0
M2MM  = 1000.0

REFERENCE_FRAME_     = 'base_0'
AR_FRAME_PREFIX_     = 'ar_marker_'
TARGET_FRAME_PREFIX_ = 'ar_target_'


OFFSET_FROM_TARGET = 175.0 * MM2M # [mm]

class snu_ar_tracker():
    def __init__(self):        
        rospy.init_node('snu_ar_tracker', anonymous=True)
        self.listener = tf.TransformListener()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = TransformStamped()

        rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_sub_cb, queue_size=1)
        
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1)

        self.tags = AlvarMarkers()
        self.cmd_pose = Pose()

    def update_pose(self, trans, rot):
        self.cmd_pose.position.x    = trans[0] - 0.020 # 보정 @(arm -> 측면)
        self.cmd_pose.position.y    = trans[1]# - 0.020 # 보정 @(arm -> 정면)
        self.cmd_pose.position.z    = trans[2]
        self.cmd_pose.orientation.x = rot[0]
        self.cmd_pose.orientation.y = rot[1]
        self.cmd_pose.orientation.z = rot[2]
        self.cmd_pose.orientation.w = rot[3]

    def pnp_cb(self, msg): # Version 2: "/tf" topic 이용
        print "@@@@@@@@@@@@@@@@@@@@@@@@@1"
        print(msg.data)
        if msg.data == 'search':
            print "@@@@@@@@@@@@@@@@@@@@@@@@@2"
            try:
                print(AR_FRAME_PREFIX_ + '1') # msg.data 나중에 이 부분 바꿔야함@@@@@@@@@@@@@@@@@@
                self.listener.waitForTransform(REFERENCE_FRAME_, TARGET_FRAME_PREFIX_ + '1', rospy.Time(), rospy.Duration(0.5)) # msg.data 나중에 이 부분 바꿔야함@@@@@@@@@@@@@@@@@@
                (trans,rot) = self.listener.lookupTransform(REFERENCE_FRAME_, TARGET_FRAME_PREFIX_ + '1', rospy.Time(0)) # msg.data 나중에 이 부분 바꿔야함@@@@@@@@@@@@@@@@@@
                #print(rot)
        

                self.update_pose(trans, rot)
                #print(self.cmd_pose)
                
                self.pub1.publish(self.cmd_pose)
                print(self.cmd_pose)

            except (tf.Exception):
                print "[ERROR]: AR Tag not detected!"
                pass
            return 1
        else:
            return -1
        

    def ar_sub_cb(self, msg): # Version 1: "ar_pose_marker" topic 이용
        n_tags = len(msg.markers)
        print "Number of detected tags: %d"%n_tags

        if n_tags is not 0:
            idx = 0
            for x in msg.markers:
                self.static_transformStamped.header.stamp    = rospy.Time.now()
                self.static_transformStamped.header.frame_id = AR_FRAME_PREFIX_ + str(msg.markers[idx].id)
                self.static_transformStamped.child_frame_id  = TARGET_FRAME_PREFIX_ + str(msg.markers[idx].id)
            
                self.static_transformStamped.transform.translation.x = 0.0
                self.static_transformStamped.transform.translation.y = 0.0
                self.static_transformStamped.transform.translation.z = OFFSET_FROM_TARGET
            
                quat = tf.transformations.quaternion_from_euler(math.pi, 0.0, math.pi/2.0)
                self.static_transformStamped.transform.rotation.x = quat[0]
                self.static_transformStamped.transform.rotation.y = quat[1]
                self.static_transformStamped.transform.rotation.z = quat[2]
                self.static_transformStamped.transform.rotation.w = quat[3]

                self.broadcaster.sendTransform(self.static_transformStamped)

                idx += 1
            return 1
        else:
            return -1
        



    
if __name__ == "__main__":
    ar = snu_ar_tracker()

    #test = String()
    #test.data = '1'
    #ar.pnp_cb(test)

    while not rospy.is_shutdown():
        #ar.pnp_cb(test)
        pass

    print 'good bye!'

