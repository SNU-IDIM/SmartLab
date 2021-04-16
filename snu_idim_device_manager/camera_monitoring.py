#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import numpy as np
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:
  def __init__(self):
    self.bridge = CvBridge()

    self.image_overview = np.empty((480, 640, 3))
    self.cam_overview_sub = rospy.Subscriber("overview/usb_cam/image_raw", Image, self.cb_overview)

    self.image_printer = np.empty((480, 640, 3))
    self.cam_printer_sub = rospy.Subscriber("printer/usb_cam/image_raw", Image, self.cb_printer)

    self.image_omm = np.empty((480, 640, 3))
    self.cam_omm_sub = rospy.Subscriber("omm/usb_cam/image_raw", Image, self.cb_omm)


  def cb_overview(self, data):
    try:
      self.image_overview = self.bridge.imgmsg_to_cv2(data, "bgr8")
      print(type(self.image_overview), self.image_overview.shape)
    except CvBridgeError as e:
      print(e)
    # cv2.imshow("Image window", self.image_overview);  cv2.waitKey(3)


  def cb_printer(self, data):
    try:
      self.image_printer = self.bridge.imgmsg_to_cv2(data, "bgr8")
      print(type(self.image_printer), self.image_printer.shape)
    except CvBridgeError as e:
      print(e)
    # cv2.imshow("Image window", self.image_printer);   cv2.waitKey(3)   


  def cb_omm(self, data):
    try:
      self.image_omm = self.bridge.imgmsg_to_cv2(data, "bgr8")
      print(type(self.image_omm), self.image_omm.shape)
    except CvBridgeError as e:
      print(e)
    # cv2.imshow("Image window", self.image_omm);   cv2.waitKey(3)   


def main(args):
  ic = ImageConverter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)