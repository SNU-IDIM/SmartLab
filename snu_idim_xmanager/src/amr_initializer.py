#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import threading, time
import sys
import math
import rospy, sys, numpy as np
from std_msgs.msg import String

NS_ = 'R_001'

if __name__=='__main__':
  rospy.init_node('amr_initializer')
  pub_amr = rospy.Publisher('/'+NS_+'/sp_routine', String, queue_size=1)
  rospy.sleep(5)
  pub_amr.publish('NAV')
  rospy.sleep(1)
  pub_amr.publish("RESET")