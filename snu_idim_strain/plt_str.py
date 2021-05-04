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

    def __init__(self,testnumber,fracture_idx):
        # t = self.testname
        try:
            start_point=50
            end_point= 350
            # end_point = fracture_idx
            # self.vision_f_name = vision_file
            self.testnumber = testnumber
            

            thickness, width= self.load_dim()
            cs_area = thickness * width
            print('area', cs_area)

            ########################################################################################################################################
            time1, load1 = self.load_instron_data() #this is from Instron
            end_point = np.shape(time1)[0]

            strain = self.load_strain_data(start_point, end_point-1) #this is from vision
            
            print(np.shape(strain))
            print(np.shape(time1))
            print(np.shape(load1))       

            resampled_time1, resampled_load1= self.sampling_interpol(time1, load1, 0.2)
            print("---------------------")
            print(np.shape(resampled_time1))
            print(np.shape(resampled_load1))

            ####################################################################
            time1 = resampled_time1[start_point:end_point]
            load1 = resampled_load1[start_point:end_point]

            smoother = ConvolutionSmoother(window_len=30, window_type='ones')
            smoother.smooth(strain)
            print("---------------------")
            print(np.shape(smoother.smooth_data))
            print(np.shape(smoother.smooth_data[0]))
            print(np.shape(load1))
            ########################################################################################################################################
            strain_len = np.shape(smoother.smooth_data[0])[0]
            load1_len = np.shape(load1)[0] 

            # short_len = min(strain_len, load1_len)

            short_len = fracture_idx - start_point -20
            time1 = time1[0:short_len]
            smooth_strain = smoother.smooth_data[0][0:short_len]
            load1 = load1[0:short_len] - load1[0]
            print("---------------------")
            print(np.shape(time1))
            print(np.shape(smooth_strain))
            print(np.shape(load1))
            
            strain_final = smooth_strain-smooth_strain[0]
            stress_final = load1/cs_area

            plt.plot(strain_final, stress_final)
            # plt.plot(strain, load1/cs_area)

            print("---------------------")

            plt.xlabel("strain (%)")
            plt.ylabel("Stress (MPa)")
            plt.title("Stress - Strain curve")
            # plt.legend(['vision'])
            # plt.show()
            plt.ioff()
            plt.savefig("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/" + self.testnumber + "/" + self.testnumber + "_plot.png")
            E = 0
            temp_strain = 0
            for i in range(0,short_len):
                if strain_final[i]> 0.7: # finding elastic modulus
                    E = stress_final[i]/0.7 *100
                    temp_strain = strain_final[i]
                    break
            print("strain value at "+str(temp_strain)+" stress value is = "+str(E))

            ulti_stress = 0
            ulti_strain = 0
            for i in range(0,short_len):
                if ulti_stress <  stress_final[i]:
                    ulti_stress  = stress_final[i]
                    ulti_strain = strain_final[i]

            print("Ultimate strain value at "+str(ulti_strain)+" stress value is = "+str(ulti_stress))
         
            ## save E, ulti_stress as txt
            filename = "C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/" + self.testnumber + "/" + self.testnumber + "_E_Su.txt"
            with open (filename, 'w') as e:
                e.write(str(round(E, 2))+',')
                e.write(str(round(ulti_stress, 2)))


            ## save stress strain data
            filename = "C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/" + self.testnumber + "/" + self.testnumber + "stress-strain-curve.xlsx"
            dat = {

                'axial_strain(%)': np.ravel(strain_final),
                'stress': np.ravel(stress_final),
            }
            df1 = pd.DataFrame(dat)
            writer = pd.ExcelWriter(filename, engine='xlsxwriter')
            df1.to_excel(writer, sheet_name='Sheet1')
            writer.close()
            
        except:
            print("plotting error")
            cv2.imwrite("C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/" + self.testnumber + "/" + self.testnumber + "_plot.png",np.zeros(10))
            filename = "C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/" + self.testnumber + "/" + self.testnumber + "_E_Su.txt"
            with open (filename, 'w') as e:
                e.write('0,0')

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
        sql2 = "SELECT Thickness,width FROM result WHERE subject_name='{}';".format(subject_name)
        cur.execute(sql1)
        cur.execute(sql2)
        con.commit()
        data = cur.fetchone()
        thickness = data['Thickness']  
        width = data['width']
        return  float(thickness), float(width)

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

# c = plotter("DRY_TEST_0",188)