import numpy as np
import cv2 as cv2
import copy
import sys
import os
# from matplotlib import pyplot as plt
import pandas as pd
# import natsort
# from scipy import signal
import time
import keyboard

#TODO: 사용법:
# 1. 이파일을 이용해서 keyboard recorder fps 를 결정할 것
# 2. 두가지 옵션 존재 1번: 키보드 s 를 누를시 프린트 되는 것을 통해 한프레임 받고, 사진 저장하고 다음 프레임을 받았을시 fps를 볼 수 있음
# 옵션 2번: 키보드 a 를 눌러서 RAM에 이미지를 저장하고 계속 진행하여 fps를 구할 수 있음 이때 print되는 값을 확인할 것

##Setup camera
cameraid = 1
cap = cv2.VideoCapture(0)
# print(cap.open(cameraid))

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("width: %d, height:%d" % (width, height))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

print("width2: %d, height2:%d" % (width, height))

cap.set(cv2.CAP_PROP_FOCUS, 200) # set focus

#define frame count and start record loop
frame_count_s = 0
frame_count = 0
img_arr = []
print("up")
while(cap.isOpened()):
    print("donw")
    ret, frame = cap.read()
    # cv2.imshow('sd',frame)
    # cv2.waitKey(1)

    if keyboard.is_pressed('s'): #checking the fps for saving every frame on each frame arrival
        frame_count_s +=1
        if frame_count_s==1:
            start_time_s = time.time()
        cv2.imwrite("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/frame/calibrate_img" + str(frame_count_s) + ".png", frame)
        if frame_count_s%10 ==0:
            print("Elapsed time is"+str(time.time()-start_time_s)+"and the frames saved is: "+ str(frame_count_s))

    if keyboard.is_pressed('a'): #checking the fps for saving each frame in RAM space
        frame_count +=1
        if frame_count==1:
            start_time =time.time()
        img_arr.append(frame)
        if frame_count%10 ==0:
            print("Elapsed time is"+str(time.time()-start_time)+"and the frames saved is: "+ str(frame_count))



'''
import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
'''