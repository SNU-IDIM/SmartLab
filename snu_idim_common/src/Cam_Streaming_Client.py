#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys;     sys.dont_write_bytecode = True
import time, json, zmq
import numpy as np
import imagezmq
from threading import Thread

class Cam_Streaming_Client(object):

    def __init__(self, ip='192.168.60.21', cam_list=['overview', 'cobot']):
        for cam_id in cam_list:
            if cam_id == 'overview':
                self.image_overview = np.empty((480, 640, 3))
                self.cam_overview = imagezmq.ImageHub('tcp://*:{}'.format(5801))
                self.thread_overview = Thread(target=self.overview_streaming)
                self.thread_overview.start()

            elif cam_id == 'cobot':
                self.image_cobot = np.empty((480, 640, 3))
                self.cam_cobot = imagezmq.ImageHub('tcp://*:{}'.format(5802))
                self.thread_overview = Thread(target=self.cobot_streaming)
                self.thread_overview.start()

    
    def overview_streaming(self):
        while True:
            rpi_name, self.image_overview = self.cam_overview.recv_image()
            print(rpi_name)
            # cv2.imshow(rpi_name, self.image_overview);  cv2.waitKey(1)
            self.cam_overview.send_reply(b'OK')


    def cobot_streaming(self):
        while True:
            rpi_name, self.image_cobot = self.cam_cobot.recv_image()
            print(rpi_name)
            # cv2.imshow(rpi_name, self.image_cobot);  cv2.waitKey(1)
            self.cam_cobot.send_reply(b'OK')


if __name__ == '__main__':

    s_client = Cam_Streaming_Client(ip='192.168.60.21', cam_list=['overview', 'cobot'])