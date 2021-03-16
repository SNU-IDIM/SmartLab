import numpy as np
import cv2 as cv
import cv2 as cv2
import copy
import sys
import os
from matplotlib import pyplot as plt
import time
import pandas as pd
import natsort
# import keyboard
import threading
#import imutils

class Instron_cam:
    def __init__(self, Device_name='instron_cam'):
        print("0")
        self.TEST_NUMBER = 1  #Define the test number
        ##set recording parameters
        self.fps = 10  #fps check after record a: 3 / s : 10
        #Debug camera frame timing.py 
        self.spf = int(1000/self.fps)  ##in milli sec


        ##Setup camera
        self.cameraid = 0
        self.cap = cv2.VideoCapture(self.cameraid + cv2.CAP_DSHOW)
        print(self.cap.open(0))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FOCUS, 130)
        self.cap.set(cv2.CAP_PROP_FPS,self.fps)

        ##setup frame counter
        self.framenum = 0
        self.start_time = time.time()
        ##setup key input
        self.start_sig = False
        self.finish_sig = 0
        
        self.button =''
        print("1")
        self.thread_2 = threading.Thread(target=self.KeyInterrupt)
        self.thread_2.daemon = True
        self.thread_2.start()
        print("2")



    def trigger(self,bullet):
        self.button = bullet

    def KeyInterrupt(self):
        while True:
            ## define variable for image array
            self.img_arr = []
            self.frameCount = 0
            print("3")
            while(self.cap.isOpened()):
                self.ret, self.frame = self.cap.read() #read camera
                print("waiting for in")
                # if keyboard.is_pressed('space'):  #for debut
                if self.button == 'recordstart': #THis is the call for appending images #change after debug
                    print("button : recordstart")

                    self.start_sig = 1
                    self.keypresstime = time.time()

                if self.start_sig == 1: #append images since space is called
                    self.img_arr.append(self.frame)
                    cv2.imshow('123',self.frame)
                    cv2.waitKey(1)
                    self.frameCount = self.frameCount + 1
                    print(self.frameCount)
                if self.button == 'recordstop': #change after debug
                # if keyboard.is_pressed('esc'):    #for debug
                    print("button : recordstop")
                    self.escape_time = time.time()
                    self.start_sig = 0
                    self.button = ''
                    break
            self.image_frame_count = np.size(self.img_arr,0)
            print("img_arr size",np.shape(self.img_arr))
            print("image fame count : ",self.image_frame_count)
            ## set path
            self.newpath = "/home/kimyun/catkin_ws/src/SNU_SmartLAB/snu_idim_strainCam/result/first" #str(subject_name)
            self.vidfilename = "/home/kimyun/catkin_ws/src/SNU_SmartLAB/snu_idim_strainCam/result/first/11.avi" #str(subject_name) + "/" + str(TEST_NUMBER) +".avi"
            if not os.path.exists(self.newpath):
                os.makedirs(self.newpath)
                print("made_directory=",self.newpath)


            ##setup video recorder
            # self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            
            self.fourcc = cv2.VideoWriter_fourcc(*'XVID')

            # out = cv2.VideoWriter('strain_result_and_video\\fps10_keyboardwrite.avi', fourcc, fps, (3840,2160))
            self.out = cv2.VideoWriter('src/SNU_SmartLAB/snu_idim_strainCam/result/first/12.avi', self.fourcc, self.fps, (1920,1080)) # set video
            # print(img_arr)
            #video make with fps in time
            for i in range(0,self.image_frame_count):
                print("currently_processing frame:"+str(i)+"out of "+str(self.image_frame_count))
                # testimg = cv2.imread('/home/kimyun/catkin_ws/src/SNU_SmartLAB/snu_idim_strainCam/image.jpg)')
                # print("img_arr :", img_arr[3])

                # print("test_img :", testimg)
                # out2.write(testimg)
                self.out.write(self.img_arr[i])
                cv2.waitKey(self.spf)

            self.out.release
            cv2.destroyAllWindows()

            print("Done writing video and frames, the total elapsed time is" +str(self.image_frame_count/10))
            print("Total frame number written is", self.image_frame_count+1)


#TODO: call a python file to automatically measure strain

# call stain_cal code by bash command

if __name__ == '__main__':

    Cam = Instron_cam()
    print("?")

    while True:
        time.sleep(1)
        print("!")
        pass