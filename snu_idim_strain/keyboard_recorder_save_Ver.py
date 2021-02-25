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

sys.path.append('C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_common/src')
import datasql

sys.path.append('C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain')
import calc_strain_

class Instron_cam:
    def __init__(self,Device_name ='instron_cam'):
        # print("0")
        self.TEST_NUMBER = 1  #Define the test number
        ##set recording parameters
        self.fps = 5  #10 is enough for long cable given max is 18 but not recommended #먼저 그냥 녹화해보고 fps를 check 해볼 것
        #Debug camera frame timing.py 이용해서 check하고 fps 설정할 것
        self.spf = int(1000/self.fps)  ##in milli sec


        ##Setup camera
        self.cameraid = 0
        self.cap = cv2.VideoCapture(self.cameraid + cv2.CAP_DSHOW)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 3840)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 2160)
        self.cap.set(cv2.CAP_PROP_FOCUS, 120)
        self.cap.set(cv2.CAP_PROP_FPS,self.fps)

        ##setup frame counter
        self.framenum = 0
        self.start_time = time.time()
        ##setup key input
        self.start_sig = False
        self.finish_sig = 0

        self.button = ''
        self.subject_name = 'base3'                 

        # print("1")
        self.thread_2 = threading.Thread(target=self.KeyInterrupt)
        self.thread_2.daemon = True
        self.thread_2.start()
        # print("2")

        self.cal_result = calc_strain_.calc_strain()
        self.sql = datasql.mysql()


    
    def trigger(self,bullet, name):
        self.subject_name = name
        self.button = bullet

    def KeyInterrupt(self):
        while True:
            ## define variable for image array
            self.img_arr = []
            self.button_flage = 1


            while(self.cap.isOpened()):


                self.ret, self.frame = self.cap.read() #read camera
                # print("waiting for in")
                # if keyboard.is_pressed('space') and self.button_flage == 1:
                if self.button =='recordstart' and self.button_flage == 1: #THis is the call for appending images
                    print("button : recordstart")
                    self.start_sig = 1
                    self.keypresstime = time.time()
                    self.button_flage = 0
                    self.frameCount = 0

                    
                    ## set path
                    self.newpath = "C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/" + str(self.subject_name + "/pics")#+str(self.TEST_NUMBER)
                    self.vidfilename = "C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/"+str(self.subject_name) + "/" +str(self.subject_name) + ".avi"

                    if not os.path.exists(self.newpath):
                        os.makedirs(self.newpath)
                        print("made_directory=",self.newpath)
                    else:  
                        print("directory exist")



                if self.start_sig ==1: #append images since space is called
                    cv2.imwrite(str(self.newpath) +"/calibrate_img" + str(self.frameCount) + ".png",self.frame)
                    # self.img_arr.append(self.frame)
                    # cv2.imshow('123',self.frame)
                    # cv2.waitKey(1)
                    self.frameCount = self.frameCount + 1
                    print(self.frameCount)
                # if keyboard.is_pressed('esc') and self.button_flage ==0:
                if self.button == 'recordstop' and self.button_flage == 0:
                    self.escape_time = time.time()
                    self.start_sig = 0
                    self.button = ''
                    self.button_flage = 1
                    break
            # self.image_frame_count = np.size(self.img_arr,0)
            # print("img_arr size",np.shape(self.img_arr))
            # print("image fame count : ",self.image_frame_count)



            ##setup video recorder
            # self.fourcc = cv2.VideoWriter_fourcc(*'WMV1') #wmv
            self.fourcc = cv2.VideoWriter_fourcc(*'MJPG') #avi
            # self.fourcc = cv2.VideoWriter_fourcc(*'XVID')# avi
            # self.fourcc = cv2.VideoWriter_fourcc(*'h264')

            # out = cv2.VideoWriter('strain_result_and_video\\fps10_keyboardwrite.avi', fourcc, fps, (3840,2160))
            self.out = cv2.VideoWriter(self.vidfilename, self.fourcc, self.fps, (1920,1080)) # set video

            #구한 fps를 이용해서 타이밍 맞춰서 영상을 제작
            for i in range(0,self.frameCount):
                print("currently_processing frame:"+str(i)+"out of "+str(self.frameCount))
                oneframe = cv2.imread(str(self.newpath) +"/calibrate_img" +str(i) + ".png", 1)

                self.out.write(oneframe)
                cv2.waitKey(self.spf)

            self.out.release
            cv2.destroyAllWindows()
            print("Done writing video and frames, the total elapsed time is" +str(self.frameCount/10))
            print("Total frame number written is", self.frameCount+1)

           
            self.cal_result.Runcal(self.subject_name)

            self.sql.send_data(self.frameCount-1, self.subject_name)

            self.subject_name = ''




#TODO: call a python file to automatically measure strain
# 여기 bash command 통해서 strain 계산코드를 부르면될 듯

if __name__ == '__main__':

    cam = Instron_cam()
    # print("?")

    while True:
        time.sleep(1)
        # print("!")
        pass