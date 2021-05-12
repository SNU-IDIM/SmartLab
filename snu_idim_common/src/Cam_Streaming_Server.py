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
	def __init__(self, ip='192.168.0.88', port=5556, cam_list=['overview', 'cobot']):
		self.bridge = CvBridge()
		self.imagezmq_sender = imagezmq.ImageSender(connect_to='tcp://{}:{}'.format(ip, port))

		for cam_id in cam_list:
			if cam_id == 'overview':
				self.image_overview = np.empty((480, 640, 3))
				self.cam_overview_sub = rospy.Subscriber("/overview/usb_cam/image_raw", Image, self.cb_overview)
			elif cam_id == 'cobot':
				self.image_cobot = np.empty((480, 640, 3))
				self.cam_robot_sub = rospy.Subscriber("/R_001/camera/color/image_rect_color", Image, self.cb_cobot)


	def cb_overview(self, data):
		try:
			self.image_overview = self.bridge.imgmsg_to_cv2(data, "bgr8")
			print(type(self.image_overview), self.image_overview.shape)
			self.imagezmq_sender.send_image('overview', self.image_overview)
		except CvBridgeError as e:
			print(e)
		# cv2.imshow("Image window", self.image_overview);  cv2.waitKey(3)


	def cb_cobot(self, data):
		try:
			self.image_cobot = self.bridge.imgmsg_to_cv2(data, "bgr8")
			print(type(self.image_cobot), self.image_cobot.shape)
			self.imagezmq_sender.send_image('cobot', self.image_cobot)
		except CvBridgeError as e:
			print(e)
		# cv2.imshow("Image window", self.image_cobot);   cv2.waitKey(3)   




if __name__ == '__main__':
	rospy.init_node('SmartLab_Cam_Streaming')

	s_server = Cam_Streaming_Server(ip='192.168.0.88', port=5556, cam_list=['overview', 'cobot'])
	
	while True:
		time.sleep(1)