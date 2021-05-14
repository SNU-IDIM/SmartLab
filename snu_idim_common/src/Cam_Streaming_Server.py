#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
import json

import numpy as np
import cv2
import imagezmq

import roslib, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Cam_Streaming_Server:
	def __init__(self, ip='192.168.60.21', cam_list=['overview', 'cobot_eef']):
		self.bridge = CvBridge()

		for cam_id in cam_list:
			if cam_id == 'overview':
				self.sender_overview = imagezmq.ImageSender(connect_to='tcp://{}:{}'.format(ip, 5801))
				self.image_overview = np.empty((480, 640, 3))
				self.cam_overview_sub = rospy.Subscriber("/overview/usb_cam/image_raw", Image, self.cb_overview)
			elif cam_id == 'cobot_eef':
				self.sender_cobot_eef = imagezmq.ImageSender(connect_to='tcp://{}:{}'.format(ip, 5802))
				self.image_cobot_eef = np.empty((480, 640, 3))
				self.cam_cobot_eef_sub = rospy.Subscriber("/R_001/camera/color/image_rect_color", Image, self.cb_cobot_eef)
			elif cam_id == 'cobot_front':
				self.sender_cobot_front = imagezmq.ImageSender(connect_to='tcp://{}:{}'.format(ip, 5803))
				self.image_cobot_front = np.empty((480, 640, 3))
				self.cam_cobot_front_sub = rospy.Subscriber("/cobot_front/usb_cam/image_raw", Image, self.cb_cobot_front)


	def cb_overview(self, data):
		try:
			self.image_overview = self.bridge.imgmsg_to_cv2(data, "rgb8")
			# print(type(self.image_overview), self.image_overview.shape)
			self.sender_overview.send_image('overview', self.image_overview)
		except CvBridgeError as e:
			print(e)
		# cv2.imshow("Image window", self.image_overview);  cv2.waitKey(3)


	def cb_cobot_eef(self, data):
		try:
			self.image_cobot_eef = self.bridge.imgmsg_to_cv2(data, "rgb8")
			# print(type(self.image_cobot_eef), self.image_cobot_eef.shape)
			self.sender_cobot_eef.send_image('cobot_eef', self.image_cobot_eef)
		except CvBridgeError as e:
			print(e)
		# cv2.imshow("Image window", self.image_cobot_eef);   cv2.waitKey(3)   

	def cb_cobot_front(self, data):
		try:
			self.image_cobot_front = self.bridge.imgmsg_to_cv2(data, "rgb8")
			# print(type(self.image_cobot_front), self.image_cobot_front.shape)
			self.sender_cobot_front.send_image('cobot_front', self.image_cobot_front)
		except CvBridgeError as e:
			print(e)
		# cv2.imshow("Image window", self.image_cobot_front);   cv2.waitKey(3)   


if __name__ == '__main__':
	rospy.init_node('SmartLab_Cam_Streaming')

	s_server = Cam_Streaming_Server(ip='192.168.60.21', cam_list=['overview', 'cobot_eef', 'cobot_front'])
	
	while True:
		time.sleep(1)