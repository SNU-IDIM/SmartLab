#!/usr/bin/env python
#-*- coding: utf-8 -*-
##################################################################################################################################################
'''
    Python Libraries
'''
import sys, os, threading, time
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_SmartLAB/snu_idim_common/imp"%HOME_DIR)) )
import timeit
import math
import numpy as np
import cv2 as cv2
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import pandas as pd
from copy import deepcopy
from math import pi
import time
import copy
from tensorflow import keras
from keras_segmentation.models.unet import vgg_unet
from keras_segmentation.models.segnet import resnet50_segnet

##################################################################################################################################################
from IDIM_framework import *

################  TO DO LIST  ################
# 1. organize parameter

################  TF NAME ################
CAMERA_FRAME_PREFIX_    = 'camera_link'
OBJECT_TARGET_PREFIX_   = 'specimen_table_'
TEMP_PREFIX_            = 'temp_'

class snu_vision_2d():
    def __init__(self):
        
        # self.model = vgg_unet(n_classes=2, input_height=448, input_width=448)
        self.model = resnet50_segnet(n_classes=2, input_height=448, input_width=448)
        # self.model.load_weights('/home/syscon/specimen_detection_vggunet/weights.h5')
        self.model.load_weights('/home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/src/weights.h5')

        self.imagewindowflag = False #False : image pass, True : image show for debugging
        self.vision_status = "filter is working"
        print(self.vision_status)
        np.set_printoptions(threshold=sys.maxsize)

    def vision(self):
        self.cv_image = None
        while True:
            if os.path.exists('/home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/specimen_image/specimen.png'):
                time.sleep(0.1)
                self.cv_image = cv2.imread('/home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/specimen_image/specimen.png' ,cv2.IMREAD_COLOR)
                self.specimen_image = copy.deepcopy(self.cv_image) # move this when you draw something in the image and change imagewindowflag
                break
            else:
                # print('there is no image file')
                time.sleep(0.3)

        if self.imagewindowflag == False :
            pass
        elif self.imagewindowflag == True:
            #Set Region of Interest
            rowEnd=634  #616 for sindoh 622 for sondori
            colEnd=448  #402 for sindoh 416 for sondori
            rowStart=186 #241 for sindoh 233 for sondori
            colStart=0 #24 for sindoh 24 for sondori

            self.specimen_cropped = self.specimen_image[colStart:colEnd, rowStart:rowEnd]

            self.unet_image = self.model.predict_segmentation(
                inp = self.specimen_cropped,
                out_fname = "/tmp/out.png"
            )
            self.specimen_unet = np.uint8(self.unet_image)
            size = np.shape(self.specimen_unet)
            for ii in range(size[0]):
                for jj in range(size[1]):
                    if self.specimen_unet[ii, jj] == 1:
                        self.specimen_unet[ii, jj] = 255

            self.edges=cv2.Canny(self.specimen_unet, 0, 200)

            # cv2.namedWindow('robot endeffector image', cv2.WINDOW_NORMAL)
            cv2.imshow('robot endeffector image', self.cv_image)  #orignal image show
            # cv2.imshow('robot endeffector image', self.bgr_image)   #cropped image show
            
            #cv2.imshow('robot unet image', self.specimen_unet)
            #cv2.imshow('robot unet image', self.edges)
            cv2.waitKey()

    def getorientation(self, pts):
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = pts[i,0]
            data_pts[i,1] = pts[i,1]
        mean = np.empty((0))
        mean, eigenvectors = cv2.PCACompute(data_pts, mean)
        cntr = (mean[0,0], mean[0,1])
        angle = math.atan2(eigenvectors[1,1], eigenvectors[1,0])
        return (angle, cntr)

    def specimen_detection(self):
        self.vision_status = "specimen_detection"
        print(self.vision_status)
        empty_csv = pd.DataFrame()
        #Set Region of Interest
        rowEnd=634  #616 for sindoh 622 for sondori
        colEnd=448  #402 for sindoh 416 for sondori
        rowStart=186 #241 for sindoh 233 for sondori
        colStart=0 #24 for sindoh 24 for sondori

        obj_count=1

        try:
            specimen = self.specimen_image
        except:
            print('NO IMAGE RECEIVED')
            return -1

        # crop image
        specimen_cropped = specimen[colStart:colEnd, rowStart:rowEnd]
        

        unet_image = self.model.predict_segmentation(
            inp = specimen_cropped,
            out_fname = "/tmp/out.png"
        )

        specimen_unet = np.uint8(unet_image)
        # print(np.shape(specimen_unet))
        # print(specimen_unet)

        size = np.shape(specimen_unet)
        print(size)
        for ii in range(size[0]):
            for jj in range(size[1]):
                if specimen_unet[ii, jj] == 1:
                    specimen_unet[ii, jj] = 255

        #Canny edge detection & Hough lines transform
        edges=cv2.Canny(specimen_unet, 0, 200)
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #image show for debugging
        # while True:
            # cv2.imshow('original', specimen)
        # cv2.imshow('cropped', specimen_cropped)
        # cv2.imshow('unet_image', specimen_unet)
        # cv2.imshow('Canny', edges)
            # cv2.imshow('Contour', contours)
        # cv2.waitKey(0)
        
        for ii in range(len(contours)):
            ptAccum=np.squeeze(contours[ii])
        
            # FILTER2 : the specimen edge should contain more than 300 points
            if (len(ptAccum) < 300) : 
                print(len(ptAccum), 'bad search : points are too many or little')
                continue
            print(len(ptAccum))

            x_Max=np.argmax(ptAccum[:,0]) # x maximum coordinate index
            x_Min=np.argmin(ptAccum[:,0]) # x minimum coordinate index
            y_Max=np.argmax(ptAccum[:,1]) # y maximum coordinate index
            y_Min=np.argmin(ptAccum[:,1]) # y minimum coordinate index

            x_Max_Filter = 0; x_Min_Filter = 0; y_Max_Filter = 0; y_Min_Filter = 0; 
            x_Max_Accum = np.empty((0,2),int); x_Min_Accum = np.empty((0,2),int); y_Max_Accum = np.empty((0,2),int); y_Min_Accum = np.empty((0,2),int); 
            # print(ptAccum)

            #FILTER2
            for ii in range(len(ptAccum)):
                # print(ptAccum[ii])
                if ptAccum[ii, 0] >= ptAccum[x_Max,0] - 3:
                    x_Max_Filter += 1
                    x_Max_Accum = np.append(x_Max_Accum, np.array([ptAccum[ii]]), axis=0)
                elif ptAccum[ii, 0] <= ptAccum[x_Min,0] + 3:
                    x_Min_Filter += 1
                    x_Min_Accum = np.append(x_Min_Accum, np.array([ptAccum[ii]]), axis=0)
                if ptAccum [ii, 1] >= ptAccum[y_Max,1] - 4:
                    y_Max_Filter += 1
                    y_Max_Accum = np.append(y_Max_Accum, np.array([ptAccum[ii]]), axis=0)
                elif ptAccum[ii, 1] <= ptAccum[y_Min,1] + 4:
                    y_Min_Filter += 1
                    y_Min_Accum = np.append(y_Min_Accum, np.array([ptAccum[ii]]), axis=0)
            
            # print(x_Max_Filter, x_Min_Filter, y_Max_Filter, y_Min_Filter)
            # print(x_Max_Accum, x_Min_Accum, y_Max_Accum, y_Min_Accum)

            if x_Max_Filter >= 40 and x_Min_Filter >= 40 and y_Max_Filter >= 40 and y_Min_Filter >= 40 :
                perpen_Flag = True
            else :
                perpen_Flag = False

            if perpen_Flag == False:
                #find four rectnagular Vertices using maximum coordinate
                x_Max_Vertice=[ptAccum[x_Max,0], ptAccum[x_Max,1]]
                x_Min_Vertice=[ptAccum[x_Min,0], ptAccum[x_Min,1]]
                y_Max_Vertice=[ptAccum[y_Max,0], ptAccum[y_Max,1]]
                y_Min_Vertice=[ptAccum[y_Min,0], ptAccum[y_Min,1]]
                vertice = [x_Min_Vertice, y_Max_Vertice, x_Max_Vertice, y_Min_Vertice]
            elif perpen_Flag == True:
                # right_Top_Vertice=[np.average(x_Max_Accum[:,0]), np.max(x_Max_Accum[:,1])]
                # left_Top_Vertice=[np.average(x_Min_Accum[:,0]), np.max(x_Min_Accum[:,1])]
                # right_Bottom_Vertice=[np.average(x_Max_Accum[:,0]), np.min(x_Max_Accum[:,1])]
                # left_Bottom_Vertice=[np.average(x_Min_Accum[:,0]), np.min(x_Min_Accum[:,1])]

                right_Top_Vertice=[np.max(y_Max_Accum[:,0]), np.average(y_Max_Accum[:,1])]
                left_Top_Vertice=[np.min(y_Max_Accum[:,0]), np.average(y_Max_Accum[:,1])]
                right_Bottom_Vertice=[np.max(y_Min_Accum[:,0]), np.average(y_Min_Accum[:,1])]
                left_Bottom_Vertice=[np.min(y_Min_Accum[:,0]), np.average(y_Min_Accum[:,1])]

                vertice=[left_Top_Vertice, right_Top_Vertice, right_Bottom_Vertice, left_Bottom_Vertice]

            # FILTER3
            vertice_np = np.array(vertice)
            edge_Length=np.zeros([4,4])
            # print(vertice_np)
            for ii in range(4):
                for jj in range(4):
                    edge_Length_Temp = np.linalg.norm(vertice_np[ii]-vertice_np[jj])
                    edge_Length[ii,jj] = edge_Length_Temp

            # print(edge_Length)
            # print(ptAccum[x_Max,0], ptAccum[x_Min,0], ptAccum[y_Max,1], ptAccum[y_Min,1])            

            print(vertice)
            # orientation1 = float(x_Max_Vertice[1]-x_Min_Vertice[1])/float(x_Max_Vertice[0]-x_Min_Vertice[0])
            # orientation2 = float(y_Max_Vertice[1]-y_Min_Vertice[1])/float(y_Max_Vertice[0]-y_Min_Vertice[0])
            # orientation1 = float(vertice[0][1]-vertice[2][1])/float(vertice[0][0]-vertice[2][0])
            # orientation2 = float(vertice[1][1]-vertice[3][1])/float(vertice[1][0]-vertice[3][0])

            # orientation1 = float(vertice[0][0]-vertice[2][0])/float(vertice[0][1]-vertice[2][1])
            # orientation2 = float(vertice[1][0]-vertice[3][0])/float(vertice[1][1]-vertice[3][1])

            # orientation = (orientation1+orientation2)/2.0
            # theta = math.atan(orientation)

            (theta, centroid) = self.getorientation(ptAccum)

            # print(theta)

            #centroid : average of all coordinates
            # centroid=[np.average(ptAccum[:,0]), np.average(ptAccum[:,1])]
            print(theta, centroid)
            #plotting for debugging 
            cv2.circle(edges_bgr, (int(centroid[0]), int(centroid[1])), 2, (0,0,255), 4)
            cv2.circle(edges_bgr, (int(vertice[0][0]), int(vertice[0][1])), 1, (0,255,255), 2)
            cv2.circle(edges_bgr, (int(vertice[1][0]), int(vertice[1][1])), 1, (0,255,255), 2)
            cv2.circle(edges_bgr, (int(vertice[2][0]), int(vertice[2][1])), 1, (0,255,255), 2)
            cv2.circle(edges_bgr, (int(vertice[3][0]), int(vertice[3][1])), 1, (0,255,255), 2)

            # while True : 
            #cv2.imshow('image', edges_bgr)
            #cv2.waitKey(3000)

            #Calibration 250mm / 551pixels -> sondori
            px2mm_Row=(centroid[0]*2.0+rowStart-320)*250.0/551.0
            px2mm_Col=(centroid[1]*2.0+colStart-240)*250.0/551.0
            print(px2mm_Row, px2mm_Col)
            empty_csv = empty_csv.append(pd.DataFrame([theta, px2mm_Row, px2mm_Col]).T)
            obj_count = obj_count+1
        
        os.remove('/home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/specimen_image/specimen.png')
        empty_csv.to_csv('/home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/specimen_image/csv/info.csv')
        self.vision_status = "vision processing complete"
        print(self.vision_status)

        ###############################################################################################################3
    
if __name__ == "__main__":
    twod_vision = snu_vision_2d()
    while True:
        start_time = time.clock()
        twod_vision.vision()
        twod_vision.specimen_detection()
        time_elapsed = time.clock() - start_time
        print('소요 시간 : %s - %s = %s' % (time.clock(), start_time, time_elapsed))
        time.sleep(0.5)
    print('good bye!')
