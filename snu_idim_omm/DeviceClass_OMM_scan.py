#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time
import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
import math
import threading
import copy
import os, sys
import json
from matplotlib import pyplot as plt
from threading import Thread

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from datasql import mysql


class DeviceClass_OMM(object):
    def __init__(self, device_name = 'omm', port_ = 'None'):
        self.port = port_
        self.device_id = device_name

        ## Device status dictionary
        self.status = dict()
        self.status['device_type'] = 'OMM'
        self.status['device_name'] = device_name
        self.status['connection'] = False
        self.status['status'] = ''
        self.status['recent_work'] = ''

        ## Result dictionary
        self.result = dict()
        self.result['subject_name'] =''
        self.result['thickness'] = 0
        self.result['length'] = 0
        self.result['width'] = 0

        self.sql = mysql(user=self.device_id, host = '192.168.60.21')

        self.thread_1 = Thread(target=self.updateStatus)
        self.thread_1.start()

    def __del__(self):
        try:
            self.thread_1.terminate()
            self.serial.close()
        except:
            pass

    def updateStatus(self):
        while True:
            # print(self.status)
            
            time.sleep(2)

##--------------------------------------------------------------------------------------------------------------------
##----------------------------------------------------need to change--------------------------------------------------
##--------------------------------------------------------------------------------------------------------------------
##---1. command list [status, connection, wake, home, measure_thickness, measure_dimension, readline, save_result]----
##--------------------------------------------------------------------------------------------------------------------
##--------------------------------------------------------------------------------------------------------------------


    def command(self, cmd_dict):
        cmd_keys = cmd_dict.keys()
        cmd_values = cmd_dict.values()

        for key in range(len(cmd_keys)):
            if cmd_keys[key] == 'status':
                self.status['status'] = cmd_dict[key]
            # elif cmd_keys[key] == 'connection':
            #     if cmd_dict[key] == True and self.status['connection'].find('Offline') != -1:
            #         self.connectDevice()
            #     elif cmd_dict[key] == False and self.status['connection'].find('Offline') == -1: 
            #         self.disconnectDevice()
            elif cmd_keys[key] == 'connection':
                if self.status['connection'] == False:
                    self.connectDevice()
                
                elif self.status['connection'] == True:
                    self.disconnectDevice()

            elif cmd_keys[key] == 'wake':
                self.wakeDevice()
            elif cmd_keys[key] == 'home':
                self.status['status'] = 'G28 : Home Position'
                self.send_home()
                # self.status['status'] = 'Idle'

                # self.send_GCode('G0 Z25')
            elif cmd_keys[key] == 'measure_thickness':
                self.status['status'] = 'G30 : Measure Thickness'
                self.send_GCode('G0 X120 Y165 Z30')
                self.send_zPosition()
                self.result['thickness'] = self.readline_zPosition()
                self.status['status'] = 'Idle'
            elif cmd_keys[key] == 'measure_dimension':
                self.status['status'] = 'Measure Size'
                
                self.send_GCode('G0 X63 Y134 Z52.1')
                print("---------------------------")
                self.cameraOn()
                time.sleep(4)

                self.capture_left = self.capture()
                self.send_GCode('G0 X142 Y134 Z52.1')
                time.sleep(4)

                self.capture_center = self.capture()
                self.send_GCode('G0 X220 Y134 Z52.1')
                time.sleep(4)

                self.capture_right = self.capture()
                self.send_GCode('G0 X143 Y220 Z240')
                self.measure_dimension()
                time.sleep(10)
                self.send_GCode('M14')
                self.status['status'] = 'Idle'
            
            elif cmd_keys[key] == 'readline':
                self.status['status'] = 'read_line'
                self.readline()
            elif cmd_keys[key] == 'save_result':
                self.result['subject_name'] = cmd_values[key]
                # result_enc = json.load(self.status)
                
                self.sql.send('smartlab_result', 'MS', self.result)
                # self.save_result(specimen_name,result_enc)
            else:
                print(" wrong command!! ")

        self.status['recent_work'] = self.status['status']
        # self.status['status'] = 'Idle'
    

    def wakeDevice(self):
        self.serial.write("\r\n\r\n") # Hit enter a few times to wake the Printrbot
        time.sleep(2)   # Wait for Printrbot to initialize
        self.serial.flushInput()  # Flush startup text in serial input

    def disconnectDevice(self):
        try:
            self.serial.close()
            self.status['connection'] = False
            print 'Measurement Station DISCONNECTED'
        except:
            self.status['connection'] = True
            pass

    def connectDevice(self):
        # try:
        self.serial = serial.Serial(port=self.port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE)
        self.status['connection'] = True
        self.status['status'] = 'Idle'
        self.send_GCode('M14')
        print("Measurement Station CONNECTED")
        # except:
        #         self.status['connection'] = False
        #         print 'Measurement Station Connection Fail'
        #         pass
            
    def send_GCode(self, gcode):
        print 'Sending GCode' + gcode
        msg = gcode
        self.serial.write(msg + '\n')

    def send_home(self):
        print 'Sending gcode : G28 -> home position'
        msg = 'G28'
        self.serial.write(msg + '\n')


    def send_zPosition(self):
        print 'Sending gcode : G30 -> Z position'
        msg = 'G30'
        self.serial.write(msg + '\n')
    
    def readline(self):
        out = self.serial.readline()
        print("readline " + out)
        return out

    def readline_zPosition(self):
        while True:
            out = self.readline()
            if out[0:3] == 'Bed':
                break
        thickness = out[-5:]
        print 'Specimen Thickness :' + str(thickness)
        thickness = thickness.split('\n')[0]
        return thickness

    def save_result(self, subn, result):
        self.status['status']='working'
        
        name = subn
        newpath = './result/'+str(name)
        filename = newpath + '/' + name + '.txt'
        if not os.path.exists(newpath):
            os.makedirs(self.newpath)
            print("made_directory=",self.newpath)


        with open(filename, 'w') as w:
            w.write(result)

    def stopsign(self, status):
        while True:
            if self.result['thickness'] != 0 and self.flag == 0:
                self.flag = 1
                break
            elif self.result['axial_length'] != 0 and self.flag == 1:
                self.flag = 2
                break
            elif self.result['width'] != 0 and self.flag == 2:
                break
        self.status['status'] = status


    
#-----------------------------------specimen size measurement---------------------------------------------

    def calibration(self,img):
        mtx = np.load('./calib/mtx.npy')
        dist = np.load('./calib/dist.npy')            
        h,  w = 1080,1920

        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        return dst

    def capture(self):
        startTime = time.time()  
        
        while True:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            color_image = self.calibration(color_image)

            # plt.imshow(color_image)
            # plt.show()
            crop_image = color_image[:,400:1400,:]

            cv2.namedWindow('RealSense',cv2.WINDOW_NORMAL)
            cv2.namedWindow('Realsense', cv2.WINDOW_AUTOSIZE)

            cv2.imshow('RealSense', crop_image)
            cv2.imshow('Realsense', np.zeros(np.shape(crop_image)))
            cv2.waitKey(1)

            finishTime = time.time()
            if finishTime - startTime > 2:
                break



        return crop_image



    def midcrop(self,crop_image):
        w = np.shape(crop_image)[1]
        mid_image = crop_image[:,w/2 - 40:w/2 + 40,:]
        return mid_image

    def center_ms(self,contour_img):
        w, h = np.shape(contour_img)[1], np.shape(contour_img)[0]
        for i in range(0,h):
            val1 = contour_img[i,int(w/2)]
            if val1 == 255:
                up_pix = i
                break

        for i in range(0,h):
            val2 = contour_img[h-i-1,int(w/2)]
            if val2 == 255:
                low_pix = h-i-1
                break
        
        width_pix_diff = low_pix - up_pix
        return width_pix_diff
            
    def color2gray(self, rawimg):
        threshold_value = 50 #120 #125
        value = 255
        hsv_img = cv2.cvtColor(rawimg, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(hsv_img)
        # plt.imshow(h)
        # plt.show()
        
        # ret, thresh_img = cv2.threshold(gray_img, threshold_value, value, cv2.THRESH_BINARY)
        ret, thresh_img = cv2.threshold(h, threshold_value, value, cv2.THRESH_BINARY)

        startTime2 = time.time()
        while True:
            
            cv2.imshow('Realsense', thresh_img)
            cv2.waitKey(1)


            finishTime2 = time.time()
            if finishTime2-startTime2 >2:
                break

        return thresh_img


    def gray2edge(self, img):
        low_threshold = 20
        high_threshold = 80
        img = img.astype(np.uint8)
        edge_img = cv2.Canny(img, low_threshold, high_threshold)
        cv2.imshow("Realsense",edge_img)
        cv2.waitKey(1)
        # time.sleep(2)

        return edge_img

    def edge2contour(self, img):
        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        count = np.shape(contours)[0]
        
        area = 0
        for i in range(count):
            new_area = cv2.contourArea(contours[i])
            # print(new_area)
            if new_area > area:
                area = new_area
                idx = i
        
        a = copy.deepcopy(contours[idx])
        b = [a]
        
        img2 = np.zeros(np.shape(img))
        # img2 = img  
        epsilon = cv2.arcLength(a, True) *0.001
        approx = cv2.approxPolyDP(a,epsilon,True)
        cv2.drawContours(img2,[approx],0,(255,255,50),cv2.FILLED)

        single_contour = [approx]


        # print("num : {}".format(len(contours)))

        cv2.imshow("Realsense", img2)
        cv2.waitKey(1)
        # plt.imshow(img2)
        # plt.show()

        return single_contour, img2



    def lefttop(self, img):
        # _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        # print(np.shape(contours[0]))
        # print(np.shape(contours[0][0]))
        contours = img


        l = np.shape(contours[0])[0]
        dist = np.zeros(l)

        for i in range(l):
            dist[i] = pow(contours[0][i][0][0],2) + pow(contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))

        print("lefttop", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def leftbottom(self, img):
        # _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = img
        
        l = np.shape(contours[0])[0]
        dist = np.zeros(l)

        for i in range(l):
            # print(contours[0][i][0][0],contours[0][i][0][1])
            dist[i] = pow(contours[0][i][0][0],2) + pow(1080 - contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))
        print("leftbottom", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def leftcenter(self, img):
        w, h = np.shape(img)[1], np.shape(img)[0]
        for i in range(0,w):
            val = img[int(h/2),i]
            if val == 255:
                left_pix = i
                break

        return left_pix


    def rightcenter(self, img):
        w, h = np.shape(img)[1], np.shape(img)[0]
        for i in range(0,w):
            val = img[int(h/2),w - i - 1]
            if val == 255:
                right_pix = w - i - 1
                break

        return right_pix

    def righttop(self, img):
        # _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = img
        
        l = np.shape(contours[0])[0]
        dist = np.zeros(l)

        for i in range(l):
            dist[i] = pow(1000 - contours[0][i][0][0],2) + pow(contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))
        print("righttop", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def rightbottom(self, img):
        # _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = img
        
        l = np.shape(contours[0])[0]
        dist = np.zeros(l)

        for i in range(l):
            dist[i] = pow(1000 - contours[0][i][0][0],2) + pow(1080 - contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))
        print("rightbottom", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def cameraOn(self):
        print("measure dimension start")
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)
        color_sensor = profile.get_device().first_color_sensor()
        color_sensor.set_option(rs.option.enable_auto_exposure, False)

    def measure_dimension(self):
        flag = 0

        ##--------------------- main -------------------------
        # try:
        #135 pixel per 13.64mm
        pix2mm= 13.3 / 13.68 * 13.5/12.75 * 165.0/371.0 * 19.3/190.0            #fix value after height fixed
        x_dir_distance = 158.0 #mm
        y_dir_distance = 30.0  #mm


        while True:
    ##------------------------left corner-----------------------
            if flag == 0:
                print("in flag")
                color = self.capture_left
                gray = self.color2gray(color)
                contour_img,_ = self.edge2contour(gray)
                # edge = self.gray2edge(contour_img)
                # edge = gray2edge(gray)

                ltop = self.lefttop(contour_img)
                lbot = self.leftbottom(contour_img)

                _,contour_img2 = self.edge2contour(gray)
                lcent = self.leftcenter(contour_img2)

                flag = 2                # for debug

        
                
    ##------------------------center --------------------------
            elif flag == 1:
                mid_color = self.midcrop(self.capture_center)
                gray = self.color2gray(mid_color)
                _,contour_img = self.edge2contour(gray)
                # edge = gray2edge(contour_img)
                width_pix = self.center_ms(contour_img)

                flag = 3                # for debug


    ##------------------------right corner-----------------------
            elif flag == 2:
                
                color = self.capture_right
                gray = self.color2gray(color)
                contour_img,_ = self.edge2contour(gray)
                
                rtop = self.righttop(contour_img)
                rbot = self.rightbottom(contour_img)

                _,contour_img2 = self.edge2contour(gray)
                rcent = self.rightcenter(contour_img2)

                flag = 1                # for debug


    #--------------------------calculate-------------------------------

            elif flag == 3:

                degree_axi = (rtop[0] - ltop[0]) * pix2mm + x_dir_distance
                degree_perp = (ltop[1] - rtop[1]) * pix2mm


                theta = math.atan2(degree_perp,degree_axi)
                # print(axial_disp_pix,perp_disp_pix, theta)
                axial_dimension = ((rcent - lcent) * pix2mm + x_dir_distance) * math.cos(theta)
                width = math.cos(theta) * width_pix* pix2mm

                
                cv2.destroyWindow('Realsense')
                cv2.destroyWindow('RealSense')
                axial_dimension = round(axial_dimension,2)
                width = round(width,2)
                print("axial dimension is {}mm, width is {}mm".format(axial_dimension,width))
                break


        # except:
        #     self.pipeline.stop()

        return axial_dimension, width


if __name__ =='__main__':
    temp = DeviceClass_OMM()
    temp.connectDevice()

    while True:
        pass





