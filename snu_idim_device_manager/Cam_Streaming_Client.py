#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, time, json, zmq
import numpy as np
import imagezmq
from threading import Thread

class Cam_Streaming_Client(object):

    def __init__(self, ip='192.168.60.21', cam_list=['overview', 'cobot']):
        self.image_overview = np.empty((480, 640, 3))
        self.image_cobot = np.empty((480, 640, 3))

        self.image_zmq = imagezmq.ImageHub('tcp://*:{}'.format(5556))
        self.thread_imagezmq = Thread(target=self.imageZmqLoop)
        self.thread_imagezmq.start()
    

    def imageZmqLoop(self):
        while True:
            rpi_name, image = self.image_zmq.recv_image()
            print(rpi_name)
            if rpi_name == 'overview':
                self.image_overview = image
            elif rpi_name == 'cobot':
                self.image_cobot = image
            # cv2.imshow(rpi_name, rpi_name);  cv2.waitKey(1)
            self.image_zmq.send_reply(b'OK')


if __name__ == '__main__':

    s_client = Cam_Streaming_Client(ip='192.168.60.21', cam_list=['overview', 'cobot'])