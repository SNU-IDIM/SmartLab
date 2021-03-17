import numpy as np
import cv2 as cv
import cv2 as cv2
import copy
import sys
import os
from matplotlib import pyplot as plt
import pandas as pd
import natsort
from scipy import signal
from scipy.signal import lfilter
from scipy.signal import savgol_filter
from tsmoothie.smoother import *


def load_MTS_data(TEST_NUMBER):

    filename_vision = "C:\\Users\\admin\\Desktop\\smartlab\\straingauge_vision\\2103data\\" + str(TEST_NUMBER) + ".xlsx"
    filename_vision = "C:\\Users\\IDIM-Instron\\Desktop\\Smart Laboratory\\"
    
    straindata = pd.read_excel(filename_vision)
    sgdata = straindata.dropna()

    df = pd.DataFrame(sgdata, columns=['Load (N)','Time (s)','Stress (MPa)', 'Strain (mm/mm)'])
    # df = df.dropna(df)
    stress = df[['Stress (MPa)']].to_numpy()
    strain = df[['Strain (mm/mm)']].to_numpy()
    load = df[['Load (N)']].to_numpy()
    time = df[['Time (s)']].to_numpy()
    stress1 = np.ravel(stress).astype(np.float)
    strain1 = np.ravel(strain).astype(np.float)

    return stress1, strain1, load, time


def interpol(t1, t2, ext1, ext2, dst):
    data_return = (ext1-ext2)/(t1-t2)*(dst-t1)+ext1
    return data_return

def sampling_interpol(time, load, sampling_time):
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
            resampled_load[index_load] = interpol(time[index_time-1], time[index_time], load[index_time-1], load[index_time], n)

        index_load = index_load + 1

    return resampled_time, resampled_load

TEST_NUMBER = 21


def load_strain_data(TEST_NUMBER,start_point, end_point):
    filename_vision = "result_1201\\sample" + str(TEST_NUMBER) + "\\sample" + str(TEST_NUMBER) + "_vision____.xlsx"
    straindata = pd.read_excel(filename_vision)
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

    strain1 = (displacement - displacement[0]) / displacement[0] * 100 +0.2
    strain2 = (displacementsmall - displacementsmall[0]) / displacementsmall[0] * 100
    return  strain1

# def moving_avg():
def add_element(numpya,N):
    for i in range(0,N-1) :
        rms = np.append(numpya,numpya[len(numpya)-1])
    return rms

def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    rm = (cumsum[N:] - cumsum[:-N]) / float(N)
    res = add_element(rm,N)
    # res = rm
    return res

def smooth_data( data, window=5):
    smoother = ConvolutionSmoother(window_len=window, window_type='ones')
    smoother.smooth(data)
    low, up = smoother.get_intervals('sigma_interval', n_sigma=3)
    return smoother.smooth_data[0], low, up

##TODO: Define variables
start_point =5
final_point = 300
#TODO: Load visual strain data
TEST_NUMBER = 25
#21. 22. 23  의경우 카메라 위치 고정 --0.96
#25,26 카메라 위치고정 --1.0
# 27,28,29 카메라위치 다름 -- 1.05

#쓸만한 자료
#22,23, 25,26,27

vision_s = []
vision_st = []
true_strain =   []
true_stress =   []


for i in range(0,5):
    idx  = [22,23,25,26,27]
    TEST_NUMBER = idx[i]
    stress ,strain ,load ,time = load_MTS_data(TEST_NUMBER)
    s8 = load_strain_data(TEST_NUMBER,start_point ,final_point)
    # stress,strain, load, time = load_MTS_data(TEST_NUMBER) #this is from vision
    resampled_time1, resampled_load1= sampling_interpol(time, load, 0.1)
    time8 = resampled_time1[start_point:final_point]
    l8 = resampled_load1[start_point:final_point]

    vision_strain,low,up = smooth_data(s8,20)
    strain_, low_, high_ = smooth_data(strain,20)

    stress_sample = stress[start_point:final_point]
    strain_s = strain_[start_point:]
    stress_s = stress[start_point:]


    vstrain = vision_strain-vision_strain[0]
    vstress = stress_sample-stress_sample[0]
    estrain = (strain_s-strain_s[0])*100*1.0
    estress = stress_s-stress_s[0]

    plt.xlabel("strain (%)")
    plt.ylabel("Stress (MPa)")
    plt.title("Stress - Strain curve")
    plt.legend(['vision based gauge', 'extensometer'])
    plt.plot(vstrain, vstress)
    plt.plot(estrain,estress)
    # plt.savefig('TEST'+str(TEST_NUMBER)+'.png')
    plt.show()
    vision_s.append(vstrain)
    vision_st.append(vstress)
    true_strain .append(estrain)
    true_stress .append(estress)

for i in range(0,5):
    if i ==1:
        continue
    plt.plot(vision_s[i], vision_st[i])
    # plt.plot(true_strain[i],true_stress[i])


plt.xlabel("strain (%)")
plt.ylabel("Stress (MPa)")
plt.title("Stress - Strain curve")
plt.legend(['vision based gauge','extensometer'])
plt.show()

# plt.plot(vision_strain-vision_strain[0],stress_sample-stress_sample[0])
# plt.plot((strain_s-strain_s[0])*100*1.0, stress_s-stress_s[0])





# plt.show()
