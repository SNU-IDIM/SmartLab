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

        self.sql = mysql(user=self.device_id, host = '192.168.0.81')

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
            print(self.status)
            
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
                # self.send_GCode('G0 Z25')
            elif cmd_keys[key] == 'measure_thickness':
                self.status['status'] = 'G30 : Measure Thickness'
                self.send_GCode('G0 X120 Y165 Z30')
                self.send_zPosition()
                self.result['thickness'] = self.readline_zPosition()
                self.status['status'] = 'Idle'
            elif cmd_keys[key] == 'measure_dimension':
                self.status['status'] = 'Measure Size'
                self.send_GCode('G0 X143 Y140 Z125')
                print("---------------------------")
                time.sleep(15)
                self.result['length'], self.result['width'] = self.measure_dimension()
                self.send_GCode('G0 X143 Y220 Z240')
                time.sleep(10)
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
        try:
            self.serial = serial.Serial(port=self.port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE)
            self.status['connection'] = True
            self.status['status'] = 'Idle'
            print("Measurement Station CONNECTED")
        except:
                self.status['connection'] = False
                print 'Measurement Station Connection Fail'
                pass
            
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
        return dst

    def capture(self):
        startTime = time.time()  
        
        while True:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())

            # plt.imshow(color_image)
            # plt.show()
            crop_image = color_image[400:800,:,:]

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
        mid_image = crop_image[:,w/2 - 20:w/2 + 20,:]
        return mid_image

    def center_ms(self,contour_img):
        w, h = np.shape(contour_img)[1], np.shape(contour_img)[0]
        for i in range(0,h):
            # print(i)
            val1 = contour_img[i,int(w/2)]
            if val1 == 255:
                up_pix = i
                break

        for i in range(0,h):
            # print(i)
            val2 = contour_img[h-i-1,int(w/2)]
            if val2 == 255:
                low_pix = h-i-1
                break
        
        width_pix_diff = low_pix - up_pix
        # print(low_pix,up_pix,width_pix_diff)
        return width_pix_diff
            
    def color2gray(self, rawimg):
        threshold_value = 90 #120 #125
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
        time.sleep(2)

        return edge_img

    def edge2contour(self, img):
        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        count = np.shape(contours)[0]
        # print(count)
        
        area = 0
        for i in range(count):
            # print(i)
            new_area = cv2.contourArea(contours[i])
            # print(new_area)
            if new_area > area:
                area = new_area
                idx = i
        
        a = copy.deepcopy(contours[idx])
        # print(np.shape(contours))
        b = [a]
        # print(np.shape(b))
        
        img2 = np.zeros(np.shape(img))
        # img2 = img  
        epsilon = cv2.arcLength(a, True) *0.001
        approx = cv2.approxPolyDP(a,epsilon,True)
        cv2.drawContours(img2,[approx],0,(255,255,50),cv2.FILLED)

        single_contour = [approx]

        # print("num : {}".format(len(contours)))

        cv2.imshow("Realsense", img2)
        cv2.waitKey(1)

        time.sleep(2)
        return single_contour, img2



    def lefttop(self, img):
        # _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        # print(np.shape(contours[0]))
        # print(np.shape(contours[0][0]))
        contours = img


        l = np.shape(contours[0])[0]
        dist = np.zeros(l)

        for i in range(l):
            # print(contours[0][i][0][0],contours[0][i][0][1])
            # print(i, contours[0][i][0])
            dist[i] = pow(contours[0][i][0][0],2) + pow(contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        # print(dist)
        cnt= dist.index(min(dist))
        print("lefttop", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def leftbottom(self, img):
        # _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = img
        
        l = np.shape(contours[0])[0]
        # print("lt", l)
        dist = np.zeros(l)

        for i in range(l):
            # print(contours[0][i][0][0],contours[0][i][0][1])
            dist[i] = pow(contours[0][i][0][0],2) + pow(400 - contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        # print(dist)
        cnt= dist.index(min(dist))
        print("leftbottom", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def righttop(self, img):
        # _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = img
        
        l = np.shape(contours[0])[0]
        # print(l)
        dist = np.zeros(l)

        for i in range(l):
            # print(contours[0][i][0][0],contours[0][i][0][1])
            dist[i] = pow(1920 - contours[0][i][0][0],2) + pow(contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))
        print("righttop", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def rightbottom(self, img):
        # _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = img
        
        l = np.shape(contours[0])[0]
        # print(l)
        dist = np.zeros(l)

        for i in range(l):
            # print(contours[0][i][0][0],contours[0][i][0][1])
            dist[i] = pow(1920 - contours[0][i][0][0],2) + pow(400 - contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))
        print("rightbottom", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def measure_dimension(self):
        print("measure dimension start")
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        # config.enable_stream(rs.stream.depth, 1920, 1080, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)
        color_sensor = profile.get_device().first_color_sensor()
        color_sensor.set_option(rs.option.enable_auto_exposure, False)
        flag = 0


        ##--------------------- main -------------------------
        try:
    
            #135 pixel per 13.64mm
            pix2mm= 13.64/136.00                 #fix value after height fixed
            x_dir_distance = 160.0 #mm
            y_dir_distance = 30.0  #mm

    


            while True:
        ##------------------------left corner-----------------------
                if flag == 0:
                    
                    color = self.capture()
                    gray = self.color2gray(color)
                    contour_img,_ = self.edge2contour(gray)
                    # edge = self.gray2edge(contour_img)
                    # edge = gray2edge(gray)

                    ltop = self.lefttop(contour_img)
                    lbot = self.leftbottom(contour_img)
                    
                    time.sleep(5)           # moving part
                    flag = 2                # for debug

            
                    
        ##------------------------center --------------------------
                elif flag == 1:
                    mid_color = self.midcrop(color)
                    gray = self.color2gray(mid_color)
                    _, contour_img = self.edge2contour(gray)
                    # edge = gray2edge(contour_img)
                    width_pix = self.center_ms(contour_img)

                    
                    time.sleep(5)           # moving part
                    flag = 3                # for debug


        ##------------------------right corner-----------------------
                elif flag == 2:

                    rtop = self.righttop(contour_img)
                    rbot = self.rightbottom(contour_img)

                    flag = 1                # for debug

        #--------------------------calculate-------------------------------

                elif flag == 3:
                    axial_disp_pix =  rtop[0] - ltop[0]
                    perp_disp_pix = ltop[1] -rtop[1]
                    theta = math.atan(perp_disp_pix/axial_disp_pix)
                    # print(rtop[0],ltop[0],rtop[1],ltop[1])
                    # print(axial_disp_pix, perp_disp_pix)
                    # print(theta,math.cos(theta))
                    x_disp = perp_disp_pix * pix2mm                             #mm
                    y_disp = axial_disp_pix * pix2mm                            #mm

                    
                    axial_dimension = math.sqrt(pow(x_disp,2) + pow(y_disp,2))
                    
                    width = pix2mm * axial_disp_pix/axial_dimension * width_pix* pix2mm
                    print(axial_dimension, width)
                    cv2.destroyWindow('Realsense')
                    cv2.destroyWindow('RealSense')
                    axial_dimension = round(axial_dimension,2)
                    width = round(width,2)
                    print("axial dimension is {}mm, width is {}mm".format(axial_dimension,width))
                    break


        except:
            self.pipeline.stop()

        return axial_dimension, width


if __name__ =='__main__':
    temp = DeviceClass_OMM()
    temp.connectDevice()

    while True:
        pass





