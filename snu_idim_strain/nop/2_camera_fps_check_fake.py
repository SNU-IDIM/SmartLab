import numpy as np
import cv2 as cv
import cv2 as cv2
import copy
import sys
import os
from matplotlib import pyplot as plt
import pandas as pd
import natsort
# from scipy import signal
import time
import keyboard


#TODO: How to use:
# 1. decide keyboard recorder fps by this file
# 2. two options exist - option 1: press 's' find fps when frame-save-frame sequence
# option 2: press 'a' find fps when RAM - save sequence

##Setup cameraid
cameraid = 0
cap = cv2.VideoCapture(cameraid+cv2.CAP_DSHOW)
print(cap.open(cameraid))


# print("width: %d, height:%d" % (width, height))
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("width2: %d, height2:%d" % (width, height))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2880)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("width2: %d, height2:%d" % (width, height))

cap.set(cv2.CAP_PROP_FOCUS, 130) # set focus
#define frame count and start record loop
frame_count_s = 0
frame_count = 0
img_arr = []

while(cap.isOpened()):
    print("1")
    ret, frame = cap.read()
    print("1.5")
    if keyboard.is_pressed('s'): #checking the fps for saving every frame on each frame arrival
        print("2")
        frame_count_s +=1
        if frame_count_s==1:
            start_time_s = time.time()
        print("The height i s"+str(height)+"width"+str(width))
        cv2.imwrite("/home/kimyun/catkin_ws/src/SNU_SmartLAB/snu_idim_strainCam/frame/calibrate_img" + str(frame_count_s) + ".png", frame)
        if frame_count_s%10 ==0:
            print("Elapsed time is"+str(time.time()-start_time_s)+"and the frames saved is: "+ str(frame_count_s))

    if keyboard.is_pressed('a'): #checking the fps for saving each frame in RAM space
        frame_count +=1
        if frame_count==1:
            start_time =time.time()
        img_arr.append(frame)
        if frame_count%10 ==0:
            print("Elapsed time is"+str(time.time()-start_time)+"and the frames saved is: "+ str(frame_count))
