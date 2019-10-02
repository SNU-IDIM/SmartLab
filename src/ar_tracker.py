#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading, time
import sys
import math
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from ar_track_alvar_msgs.msg import *
from tf.transformations import *

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

MM2M = 1.0 / 1000.0
M2MM  = 1000.0

REFERENCE_FRAME_ = 'base_0'
AR_FRAME_PREFIX_ = 'ar_marker_'


OFFSET_FROM_TARGET = 150.0 * MM2M # [mm]

class snu_ar_tracker():
    def __init__(self):        
        rospy.init_node('snu_ar_tracker', anonymous=True)
        self.listener = tf.TransformListener()

        rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1)
        #rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_sub_cb, queue_size=1)
        
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1)

        self.tags = AlvarMarkers()
        self.cmd_pose = Pose()

    def update_pose(self, trans, rot):
        self.cmd_pose.position.x    = trans[0]
        self.cmd_pose.position.y    = trans[1] - 0.020
        self.cmd_pose.position.z    = trans[2] + OFFSET_FROM_TARGET
        #self.cmd_pose.orientation.x = rot[0]
        #self.cmd_pose.orientation.y = rot[1]
        #self.cmd_pose.orientation.z = rot[2]
        #self.cmd_pose.orientation.w = rot[3]

    def pnp_cb(self, msg): # Version 2: "/tf" topic 이용
        try:
            #print(AR_FRAME_PREFIX_ + msg.data)
            self.listener.waitForTransform(REFERENCE_FRAME_, AR_FRAME_PREFIX_ + msg.data, rospy.Time(), rospy.Duration(0.5))
            (trans,rot) = self.listener.lookupTransform(REFERENCE_FRAME_, AR_FRAME_PREFIX_ + msg.data, rospy.Time(0))
            #print(trans)
       

            self.update_pose(trans, rot)
            print(self.cmd_pose)
            
            self.pub1.publish(self.cmd_pose)

        except (tf.Exception):
            print "[ERROR]: AR Tag not detected!"
            pass
        
'''
    def ar_sub_cb(self, msg): # Version 1: "ar_pose_marker" topic 이용
        n_tags = len(msg.markers)
        #print "Number of detected tags: %d"%n_tags

        if n_tags is not 0:
            idx = 0
            for x in msg.markers:
                if msg.markers[idx].id == 1:
                    self.cmd_pose = msg.markers[idx].pose.pose
                    self.cmd_pose.position.x += OFFSET_FROM_TARGET
                    self.cmd_pose.orientation
                    self.pub1.publish(self.cmd_pose)
                    return 1
                idx += 1
        else:
            return -1
        
'''


    
if __name__ == "__main__":
    ar = snu_ar_tracker()

    test = String()
    test.data = '1'
    ar.pnp_cb(test)

    while not rospy.is_shutdown():
        ar.pnp_cb(test)
        pass

    print 'good bye!'

