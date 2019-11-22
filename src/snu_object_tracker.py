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


OFFSET_FROM_TARGET = 175.0 * MM2M # [mm]

class snu_object_tracker():
    def __init__(self):
        rospy.init_node('snu_obejct_tracker', anonymous=True)
        
        rospy.set_param("snu_object_tracker/reference_frame", 'base_0')
        rospy.set_param("snu_object_tracker/object_frame", 'ar_marker_1')
        rospy.set_param("snu_object_tracker/target_frame", 'target_1')

        self.listener = tf.TransformListener()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = TransformStamped()

        ### Class Variables ###
        self.reference_frame_name = rospy.get_param("snu_object_tracker/reference_frame", 'base_0')
        self.object_frame_name    = rospy.get_param("snu_object_tracker/object_frame", 'ar_marker_1')
        self.target_frame_name    = rospy.get_param("snu_object_tracker/target_frame", 'target_1')
        self.tags = AlvarMarkers()
        self.cmd_pose = Pose()

        ### Topics to Subscribe ###
        rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1) # Trigger Topic
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.object_sub_cb, queue_size=1) # Object Pose Topic
        
        ### Topics to Publish ###
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1) # Final target pose to be tracked


    def update_cmd_pose(self, trans, rot):
        self.cmd_pose.position.x    = trans[0] - 0.020 # 보정 @(arm -> 측면)
        self.cmd_pose.position.y    = trans[1]# - 0.020 # 보정 @(arm -> 정면)
        self.cmd_pose.position.z    = trans[2]
        self.cmd_pose.orientation.x = rot[0]
        self.cmd_pose.orientation.y = rot[1]
        self.cmd_pose.orientation.z = rot[2]
        self.cmd_pose.orientation.w = rot[3]


    def pnp_cb(self, msg): # Version 2: "/tf" topic 이용
        print "/ur_pnp -> %s"%msg.data
        if msg.data == 'search':
            # Update the Target Frame Name (임시로 rosparam 이용해 target_frame_name 수정 - SMACH 구성 후 수정 필요)
            self.object_frame_name = rospy.get_param("snu_object_tracker/object_frame_name")
            print "Searching for the target: %s ..."%self.target_object_frame_name
            try:
                print(self.target_object_frame_name)
                self.listener.waitForTransform(self.reference_frame_name, self.object_frame_name, rospy.Time(), rospy.Duration(0.5))
                (trans,rot) = self.listener.lookupTransform(self.reference_frame_name, self.object_frame_name, rospy.Time(0))

                self.update_cmd_pose(trans, rot)
                self.pub1.publish(self.cmd_pose)
                print(self.cmd_pose)

            except (tf.Exception):
                print "[ERROR]: The Object(TF) is not Detected !!!"
                pass
            return 1
        else:
            return -1
        

    def object_sub_cb(self, msg): # 받는 msg 바꿔야 함
        n_tags = len(msg.markers)
        print "Number of detected tags: %d"%n_tags

        if n_tags is not 0:
            idx = 0
            for x in msg.markers:
                self.static_transformStamped.header.stamp    = rospy.Time.now()
                self.static_transformStamped.header.frame_id = OBJECT_FRAME_PREFIX_ + str(msg.markers[idx].id)
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
    ObjectTracker = snu_object_tracker()

    #test = String()
    #test.data = '1'
    #ar.pnp_cb(test)

    while not rospy.is_shutdown():
        #ar.pnp_cb(test)
        pass

    print 'good bye!'

