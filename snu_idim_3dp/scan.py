import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
import threading
from matplotlib import pyplot as plt
# import keyboard


class scan_specimen():
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # config.enable_stream(rs.stream.depth, 1920, 1080, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)
        self.flag = 0
        # self.liveview()
        self.scan_start(160)
        print("arrived")


    def liveview(self):
        while True:
            frames = self.pipeline.wait_for_frames()

            live_frame = frames.get_color_frame()

            live_image = np.asanyarray(live_frame.get_data())
            live_crop_image = live_image#[140:350,250:350,:]

            cv2.namedWindow('liveview',cv2.WINDOW_AUTOSIZE)

            cv2.imshow('liveview', live_crop_image)
            cv2.waitKey(1)


    def capture(self):
        startTime = time.time()  
        print("where")
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            color_image = np.asanyarray(color_frame.get_data())
            crop_image = color_image #[140:350,250:350,:]
            # crop_image = color_image[480:680,550:1500,:]

            cv2.namedWindow('RealSense',cv2.WINDOW_NORMAL)
            cv2.namedWindow('Realsense', cv2.WINDOW_AUTOSIZE)
            # crop_image = live_crop_image
            cv2.imshow('RealSense', crop_image)
            cv2.imshow('Realsense', np.zeros(np.shape(crop_image)))
            # time.sleep(2)
            cv2.waitKey(1)
            # print("the size is : {}".format(np.shape(crop_image)))

            finishTime = time.time()
            if finishTime - startTime > 2:
                # cv2.destroyWindow(RealSense)
                break

        print("5")

        return crop_image

    def color2gray(self, rawimg):
        print("1")
        threshold_value = 100 #120 #125
        value = 255
        gray_img = cv2.cvtColor(rawimg, cv2.COLOR_BGR2GRAY)

    #---------------------higher view-----------------------------------------

        # plt.imshow(gray_img)
        # b,g,r = cv2.split(rawimg)
        # plt.imshow(b)
        # plt.show()
        # gb, threshimg_1 = cv2.threshold(gray_img, 160, value, cv2.THRESH_BINARY)
        # plt.imshow(threshimg_1)
        # plt.show()
    #-------------------------------------------------------------------------
        
        ret, thresh_img = cv2.threshold(gray_img, threshold_value, value, cv2.THRESH_BINARY)

        startTime2 = time.time()
        while True:
            # print("2")
            
            # cv2.imshow('threshole', thresh_img)
            cv2.imshow('Realsense', thresh_img)
            cv2.waitKey(1)


            finishTime2 = time.time()
            if finishTime2-startTime2 >2:
                # cv2.destroyWindow(threshold)
                break
        

        # return thresh_img 
        return thresh_img

    def sharpen(self, img):
        kernel = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])
        # image = cv2.resize(img,(256,256))
        sharpen_img = cv2.filter2D(img, -1, kernel)
        cv2.imshow('RealSense',sharpen_img)
        cv2.waitKey(1)
        time.sleep(2)

        return(sharpen_img)

    def gray2edge(self, img):
        low_threshold = 20 #20
        high_threshold = 80 #80
        print("start")
        edge_img = cv2.Canny(img, low_threshold, high_threshold)
        cv2.imshow("Realsense",edge_img)
        cv2.imshow("Realsense",edge_img)
        cv2.imshow("Realsense",edge_img)
        cv2.imshow("Realsense",edge_img)
        cv2.imshow("Realsense",edge_img)
        cv2.imshow("Realsense",edge_img)
        cv2.waitKey(1)
        time.sleep(2)
        print("done")

        return edge_img

    def edge2contour(self, img):
        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        img2 = img

    ##----------------polygon---------------------------------------------
        # x, y ,w, h = cv2.boundingRect(contour)
        # print(x,y,w,h)
        
        # # approx = cv2.minAreaRect(img)

        # cv2.rectangle(img2,(x,y),(x+w,y+h),(0,0,255),3)
        # rect = cv2.minAreaRect(contour)
        # box = cv2.boxPoints(rect)
        # box = np.int0(box)
        # print(box)


        # # plt.imshow(im,'gray')
        # # plt.show()
    ##---------------------------------------------------------------



    ##----------------convex-----------------------------------------

        for i in contours:
            hull = cv2.convexHull(i, clockwise=True)
            cv2.drawContours(img2,[hull],0,(0,0,255),2)

    #-------------------------------------------------------------

        # img2 = cv2.drawContours(img2,[box],0,(0,255,0),3)

        # cv2.drawContours(img2, [approx], -1, (0, 0, 255), 4)

        print("num : {}".format(len(contours)))

        cv2.imshow("Realsense", img2)
        cv2.imshow("Realsense", img2)
        cv2.imshow("Realsense", img2)
        cv2.imshow("Realsense", img2)
        cv2.imshow("Realsense", img2)
        cv2.imshow("Realsense", img2)
        cv2.imshow("Realsense", img2)
        cv2.waitKey(1)

        time.sleep(2)
        print("finish")
        return img2

    def lefttop(self, img):
        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        l = np.shape(contours[0])[0]
        dist = np.zeros(l)

        for i in range(l):
            # print(contours[0][i][0][0],contours[0][i][0][1])
            dist[i] = pow(contours[0][i][0][0],2) + pow(contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))
        print("lefttop", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def leftbottom(self, img):

        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        l = np.shape(contours[0])[0]
        dist = np.zeros(l)

        for i in range(l):
            # print(contours[0][i][0][0],contours[0][i][0][1])
            dist[i] = pow(contours[0][i][0][0],2) + 210 - pow(contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))
        print("leftbottom", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def righttop(self, img):
        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        l = np.shape(contours[0])[0]
        dist = np.zeros(l)

        for i in range(l):
            # print(contours[0][i][0][0],contours[0][i][0][1])
            dist[i] = pow(100 - contours[0][i][0][0],2) + pow(contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))
        print("righttop", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def rightbottom(self, img):
        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        l = np.shape(contours[0])[0]
        dist = np.zeros(l)

        for i in range(l):
            # print(contours[0][i][0][0],contours[0][i][0][1])
            dist[i] = pow(100 - contours[0][i][0][0],2) + 210 - pow(contours[0][i][0][1] ,2)
        
        dist = map(int, dist)
        cnt= dist.index(min(dist))
        print("rightbottom", str(contours[0][cnt][0]))

        return contours[0][cnt][0]

    def scan_start(self, move_distance):
        print("finally@@")
##--------------------- main -------------------------
        try:
            #108.5 pixel per 13.64mm
            pix2mm= 13.64/108.5                 #fix value after height fixed
            x_dir_distance = move_distance #mm
            # y_dir_distance = 30.0  #mm
            print("we")

            while True:
        ##------------------------left corner-----------------------
                if self.flag == 0:
                    print("are")
                    color = self.capture()
                    gray = self.color2gray(color)
                    contour_img = self.edge2contour(gray)
                    edge = self.gray2edge(contour_img)


                    ltop = self.lefttop(edge)
                    lbot = self.leftbottom(edge)
                    
                    time.sleep(5)           # moving part
                    self.flag = 0                # for debug
                    print("here")
            
                    
        ##------------------------center --------------------------
                elif self.flag == 1:
                    color = self.capture()
                    gray = self.color2gray(color)
                    contour_img = self.edge2contour(gray)
                    edge = self.gray2edge(contour_img)


                    cltop = self.lefttop(edge)
                    clbot = self.leftbottom(edge)
                    crtop = self.righttop(edge)
                    crbot = self.rightbottom(edge)

                    
                    time.sleep(5)           # moving part
                    self.flag = 2                # for debug


        ##------------------------right corner-----------------------
                elif self.flag == 2:
                    color = self.capture()
                    gray = self.color2gray(color)
                    contour_img = self.edge2contour(gray)
                    edge = self.gray2edge(contour_img)


                    # rtop = lefttop(right_edge)
                    # rbot = leftbottom(left_edge)
                    rtop = self.righttop(edge)
                    rbot = self.rightbottom(edge)
                    
                    # time.sleep(5)           # moving part
                    self.flag = 3                # for debug

        #--------------------------calculate-------------------------------

                elif self.flag == 3:
                    axial_disp_pix = (100 - ltop[0]) + rtop[0]
                    perp_disp_pix = rtop[1] - ltop[1]
                    
                    x_disp = perp_disp_pix * pix2mm + x_dir_distance            #mm
                    y_disp = axial_disp_pix * pix2mm                            #mm
                    
                    axial_dimension = math.sqrt(pow(x_disp,2) + pow(y_disp,2))

                    width = ( (clbot[1] + crbot[1])/2 - (cltop[1] + crtop[1])/2 ) * pix2mm

                    print("axial dimension is {}mm, width is {}mm".format(axial_dimension,width))
                    break

        except:
        #     # Stop streaming
            self.pipeline.stop()

if __name__ =='__main__':
    scan = scan_specimen()
    print("late")
