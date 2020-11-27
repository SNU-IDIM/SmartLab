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

REFERENCE_FRAME_ = 'camera_link'
AR_FRAME_PREFIX_ = 'ar_marker_'


OFFSET_FROM_TARGET = 100.0 * MM2M # [mm]

class snu_ar_tracker():
    def __init__(self):        
        rospy.init_node('snu_ar_tracker', anonymous=True)
        self.listener = tf.TransformListener()

        rospy.Subscriber('ur_pnp', String, self.pnp_cb, queue_size=1)
        
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1)

        self.tags = AlvarMarkers()
        self.cmd_pose = Pose()

    def update_pose(self, trans, rot):
        self.cmd_pose.position.x    = trans[0] + OFFSET_FROM_TARGET
        self.cmd_pose.position.y    = trans[1]
        self.cmd_pose.position.z    = trans[2]
        self.cmd_pose.orientation.x = rot[0]
        self.cmd_pose.orientation.y = rot[1]
        self.cmd_pose.orientation.z = rot[2]
        self.cmd_pose.orientation.w = rot[3]


    def pnp_cb(self, msg): # Version 2: "/tf" topic 이용
        try:
            print(AR_FRAME_PREFIX_ + msg.data)
            self.listener.waitForTransform(AR_FRAME_PREFIX_ + msg.data, REFERENCE_FRAME_, rospy.Time(), rospy.Duration(0.5))
            (trans,rot) = self.listener.lookupTransform(AR_FRAME_PREFIX_ + msg.data, REFERENCE_FRAME_, rospy.Time(0))

            self.update_pose(trans, rot)
            print(self.cmd_pose)
            self.pub1.publish(self.cmd_pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "[ERROR]: AR Tag not detected!"
            pass
        
'''
    def pnp_cb(self, msg): # Version 1: "ar_pose_marker" topic 이용
        n_tags = len(msg.markers)
        print "Number of detected tags: %d"%n_tags

        if n_tags is not 0:
            idx = 0
            for x in self.tags.markers:
                if self.tags.markers[idx].id is int(msg):
                    return idx
                idx += 1
        else:
            return -1
        
        self.cmd_pose = self.tags.markers[idx].pose.pose
        self.cmd_pose.position.x += OFFSET_FROM_TARGET
        #self.cmd_pose.orientation

        pub1.publish(self.cmd_pose.position)
'''

    
if __name__ == "__main__":
    ar = snu_ar_tracker()

    while not rospy.is_shutdown():
        pass

    print 'good bye!'
