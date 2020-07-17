#!/usr/bin/env python
from __future__ import print_function
import rospy

# import roslibroslib.load_manifest('my_package')
import sys
import os
import threading, time
import random
import copy
import math
import matplotlib.pyplot as plt


#vision related import functions
import cv2 as cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import pyrealsense2 as rs
from distutils.version import LooseVersion

class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    ##currently subscribing in 640 *480 color image
    #BGR segmentation
    #Yellow is usually RGB [255,255,0]
    #BGR = [0,255,255]

    blue_thres_up = 80
    blue_thres_down = 45

    green_thres_up = 150
    green_thres_down = 125

    red_thres_up = 180
    red_thres_down = 150

    bgr_thres_up = [blue_thres_up, green_thres_up, red_thres_up]
    bgr_thres_down = [blue_thres_down, green_thres_down, red_thres_down]


    # color_im_c = copy.deepcopy(cv_image)
    # pixel_x = []
    # pixel_y = []
    # pixel_val = []
    # for x in range(0,np.size(color_im_c,0)): ##size is 480
    #         for y in range(0,np.size(color_im_c,1)): ##size is 640
    #             if(color_im_c[x, y, 0] > bgr_thres_down[0] and color_im_c[x,y, 0] < bgr_thres_up[0] and
    #             color_im_c[x,y,1] > bgr_thres_down[1] and color_im_c[x,y, 1] < bgr_thres_up[1] and
    #             color_im_c[x,y,2] > bgr_thres_down[2] and color_im_c[x,y, 2] < bgr_thres_up[2] ):
    #                 color_im_c[x,y,0] = 255
    #                 color_im_c[x,y,1] = 255
    #                 color_im_c[x,y,2] = 255
                    
    #                 temp = [x,y]
    #                 pixel_x.append(x)
    #                 pixel_y.append(y)
    #                 pixel_val.append(temp)
    #             else:
    #                 color_im_c[x,y,0] = 0
    #                 color_im_c[x,y,1] = 0
    #                 color_im_c[x,y,2] = 0

    
    cv2.imshow('image', cv_image)
    cv2.waitKey(1)
    # x_min = min(pixel_y)
    # x_max = max(pixel_y)
    # y_min = min(pixel_x)
    # y_max = max(pixel_x)

    # print('x_min is',x_min,'xmax is', x_max)
    # print('y_min is',y_min,'ymax is', y_max)

    
    # #Line drawing line in green function for debugging
    # cv2.line(cv_image, (x_min,y_min), (x_min,y_max), (0, 255, 0), 3)
    # cv2.line(cv_image, (x_min,y_min), (x_max,y_min), (0, 255, 0), 3)
    # cv2.line(cv_image, (x_min,y_max), (x_max,y_max), (0, 255, 0), 3)
    # cv2.line(cv_image, (x_max,y_min), (x_max,y_max), (0, 255, 0), 3)
    # cv2.imshow('detected', cv_image)
    # cv2.waitKey(1)
    
    
    

    
    

    
    
    

    
    

    



def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
