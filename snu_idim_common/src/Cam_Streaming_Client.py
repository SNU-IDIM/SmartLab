#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, time, json, zmq
import numpy as np
import imagezmq
from threading import Thread

class Cam_Streaming_Client(object):

    def __init__(self, ip='192.168.60.21', cam_list=['overview', 'cobot_eef', 'cobot_front']):
        for cam_id in cam_list:
            if cam_id == 'overview':
                self.image_overview = np.empty((480, 640, 3))
                self.cam_overview = imagezmq.ImageHub('tcp://*:{}'.format(5801))
                self.thread_overview = Thread(target=self.overview_streaming)
                self.thread_overview.start()

            elif cam_id == 'cobot_eef':
                self.image_cobot_eef = np.empty((480, 640, 3))
                self.cam_cobot_eef = imagezmq.ImageHub('tcp://*:{}'.format(5802))
                self.thread_cobot_eef = Thread(target=self.cobot_eef_streaming)
                self.thread_cobot_eef.start()

            elif cam_id == 'cobot_front':
                self.image_cobot_front = np.empty((480, 640, 3))
                self.cam_cobot_front = imagezmq.ImageHub('tcp://*:{}'.format(5803))
                self.thread_cobot_front = Thread(target=self.cobot_front_streaming)
                self.thread_cobot_front.start()
    
    def overview_streaming(self):
        while True:
            rpi_name, self.image_overview = self.cam_overview.recv_image()
            # print(rpi_name)
            # cv2.imshow(rpi_name, self.image_overview);  cv2.waitKey(1)
            self.cam_overview.send_reply(b'OK')


    def cobot_eef_streaming(self):
        while True:
            rpi_name, self.image_cobot_eef = self.cam_cobot_eef.recv_image()
            # print(rpi_name)
            # cv2.imshow(rpi_name, self.image_cobot_eef);  cv2.waitKey(1)
            self.cam_cobot_eef.send_reply(b'OK')

    def cobot_front_streaming(self):
        while True:
            rpi_name, self.image_cobot_front = self.cam_cobot_front.recv_image()
            # print(rpi_name)
            # cv2.imshow(rpi_name, self.image_cobot_eef);  cv2.waitKey(1)
            self.cam_cobot_front.send_reply(b'OK')


if __name__ == '__main__':

    s_client = Cam_Streaming_Client(ip='192.168.60.21', cam_list=['overview', 'cobot_eef', 'cobot_front'])