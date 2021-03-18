import numpy as np
import cv2 as cv
import cv2 as cv2
import copy
import sys
import os
from matplotlib import pyplot as plt
import pandas as pd
# import natsort
from tsmoothie.smoother import *
import csv
import pymysql
import pandas as pd
from PIL import Image
import base64
from io import BytesIO
import time


class plotter:

    def __init__(self,testnumber):
        # t = self.testname
        start_point=10
        end_point= 270
        # self.vision_f_name = vision_file
        self.testnumber = testnumber
        

        thickness= self.load_dim()
        cs_area = thickness

        ########################################################################################################################################
        strain = self.load_strain_data(start_point, end_point)
        time1, load1 = self.load_instron_data() #this is from vision
               
        resampled_time1, resampled_load1= self.sampling_interpol(time1, load1, 0.2)

        ####################################################################
        time1 = resampled_time1[start_point:end_point]
        load1 = resampled_load1[start_point:end_point]

        smoother = ConvolutionSmoother(window_len=30, window_type='ones')
        smoother.smooth(strain)

        ########################################################################################################################################
        plt.plot(smoother.smooth_data[0], load1/cs_area)
        # plt.plot(strain, load1/cs_area)

        plt.xlabel("strain (%)")
        plt.ylabel("Stress (MPa)")
        plt.title("Stress - Strain curve")
        # plt.legend(['vision'])
        plt.show()

    def load_dim(self):
        user='IDIM-Instron'
        host='192.168.60.21'
        port = 3306
        password='0000'
        db='SmartLab'
        charset='utf8'

        con = pymysql.connect(user=user, host=host, port=port, password=password,db=db,charset=charset)
        cur = con.cursor(pymysql.cursors.DictCursor)
        subject_name = self.testnumber

        sql1 = "USE SmartLab"
        sql2 = "SELECT * FROM result WHERE subject_name='{}';".format(subject_name)
        cur.execute(sql1)
        cur.execute(sql2)
        con.commit()
        data = cur.fetchone()
        thickness = data['Thickness']
        #width = data['Width']
        #, float(width)
        return  float(thickness)

    # load_dim()
    def load_instron_data(self):

        # filename_vision = self.instron_f_name

        data = pd.read_csv("C:/Users/IDIM-Instron/Desktop/Smart Laboratory/" + str(self.testnumber) +".is_tens_RawData/Specimen_RawData_1.csv" , skiprows=8, thousands=',')
        df = pd.DataFrame(data, columns=['Time', 'Extension', 'Load'])
        df = df.drop([0])
        df['Load'] = df['Load'].str.replace(',', '').astype(
            float)  # time difference between data  s1: 5초차이   s2: 1초 차이    s3:  3.8초    s5:
        time = df[['Time']].to_numpy()
        load = df[['Load']].to_numpy()

        time = np.ravel(time).astype(float)
        load = np.ravel(load).astype(float)

        return time, load
    # time, load = load_instron_data()

    def interpol(self,t1, t2, ext1, ext2, dst):
        data_return = (ext1-ext2)/(t1-t2)*(dst-t1)+ext1
        return data_return

    def sampling_interpol(self,time, load, sampling_time):
        index_time = 0
        index_load = 0
        number_of_data = int(max(time)/0.1+1)
        resampled_time = np.arange(0, max(time), 0.1)
        resampled_load = np.empty(number_of_data)

        for n in resampled_time:
            while n > time[index_time]:
                index_time = index_time +1

            if n == time[index_time]:
                resampled_load[index_load] = load[index_time]
            elif n < time[index_time]:
                resampled_load[index_load] = self.interpol(time[index_time-1], time[index_time], load[index_time-1], load[index_time], n)

            index_load = index_load + 1

        return resampled_time, resampled_load


    def load_strain_data(self,start_point, end_point):
        filename = "C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/" + self.testnumber + "/" + self.testnumber + "__vision___.xlsx"
        # filename_vision = self.instron_f_name
        straindata = pd.read_excel(filename)
        sgdata = straindata.dropna()

        axialsg = sgdata[['axial_strain(%)']].to_numpy()
        displacement = sgdata[['displacement']].to_numpy()
        dispsmall = sgdata[['dispsmall']].to_numpy()

        axialsg_ = axialsg[0:len(axialsg)].reshape(-1, 1)
        displacement_ = displacement[0:len(displacement)].reshape(-1, 1)
        dispsmall_ = dispsmall[0:len(dispsmall)].reshape(-1, 1)

        axialsg_ = axialsg[0:len(axialsg)].reshape(-1, 1)
        displacement_ = displacement[0:len(displacement)].reshape(-1, 1)
        dispsmall_ = dispsmall[0:len(dispsmall)].reshape(-1, 1)

        displacement = displacement_[start_point:end_point]
        displacementsmall = dispsmall_[start_point:end_point]

        strain1 = (displacement - displacement[0]) / displacement[0] * 100
        strain2 = (displacementsmall - displacementsmall[0]) / displacementsmall[0] * 100

        return  strain1

    # print(strain)

    def add_element(self,numpya,N):
        for i in range(0,N-1) :
            rms = np.append(numpya,numpya[len(numpya)-1])
        return rms

    def running_mean(self,x, N):
        cumsum = np.cumsum(np.insert(x, 0, 0))
        rm = (cumsum[N:] - cumsum[:-N]) / float(N)
        res = add_element(rm,N)
        # res = rm
        return res

    ########################################################################################################################################

# c = plotter("test2")