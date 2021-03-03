import numpy as np
import cv2 as cv
import cv2 as cv2
import copy
import sys
import os
# from matplotlib import pyplot as plt
import time
import pandas as pd
# import natsort
import keyboard
import threading

fps = 5  #10 is enough for long cable given max is 18 but not recommended #먼저 그냥 녹화해보고 fps를 check 해볼 것
spf = int(1000/fps)  ##in milli sec


        ##Setup camera
cameraid = 0
cap = cv2.VideoCapture(cameraid + cv2.CAP_DSHOW)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 2160)
cap.set(cv2.CAP_PROP_FOCUS, 120)
cap.set(cv2.CAP_PROP_FPS,fps)

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("width2: %d, height2:%d" % (width, height))


        ##setup frame counter
framenum = 0
        ##setup key input
 
button = ''
newpath = "C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/frame/"
# vidfilename = "C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/frame/calibimg_" + str(frameCount) + ".avi"

while True:
    ## define variable for image array
    frame_count_a = 0

    while(cap.isOpened()):


        _, frame = cap.read() #read camera
            # one by one
        if keyboard.is_pressed('a'):
            if frame_count_a == 0:
                start_time_a = time.time()
            cv2.imwrite(newpath + "calibimg_" + str(frame_count_a) + ".png",frame)
            frame_count_a +=1
            if frame_count_a%10 ==0:
                print("Elapsed time is" + str(time.time()-start_time_a)+"and the frames saved is: "+ str(frame_count_a))

