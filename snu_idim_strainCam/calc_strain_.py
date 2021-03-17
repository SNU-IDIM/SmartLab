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


mm_per_pix = 0.08
mm_per_pix = 2 / 31.7
round_n = 6

flag = 0
initial_axial = 0
initial_perp = 0


def crop_image(image, x, w, y, h):
    return image[y:y + h, x:x + w]


def bgr2rgb(img):
    return cv.cvtColor(img, cv.COLOR_BGR2RGB)

def displayplt(img):
    plt.imshow(bgr2rgb(img))
    plt.show()

def displaygplt(img):
    plt.imshow(img, cmap='gray', vmin=0, vmax=255)
    plt.show()

def calculate_displacement(centroid):  ##returns mm
    a = sorted(centroid, key=lambda a_entry: a_entry[0])
    # print(a)
    axial1 = round(centroid[1][1] - centroid[0][1], round_n)
    axial2 = round(centroid[2][1] - centroid[1][1], round_n)
    axial3 = round(centroid[4][1] - centroid[3][1], round_n)
    axial4 = round(centroid[5][1] - centroid[4][1], round_n)
    axial5 = round(centroid[7][1] - centroid[6][1], round_n)
    axial6 = round(centroid[8][1] - centroid[7][1], round_n)

    perp1 = round(centroid[3][0] - centroid[0][0], round_n)
    perp2 = round(centroid[4][0] - centroid[1][0], round_n)
    perp3 = round(centroid[5][0] - centroid[2][0], round_n)
    perp4 = round(centroid[6][0] - centroid[3][0], round_n)
    perp5 = round(centroid[7][0] - centroid[4][0], round_n)
    perp6 = round(centroid[8][0] - centroid[5][0], round_n)
    dist_arr = [[axial1, perp1], [axial2, perp2], [axial3, perp3], [axial4, perp4], [axial5, perp5], [axial6, perp6]]
    axial_sum = 0
    perp_sum = 0
    for i in range(0, np.size(dist_arr, 0)):
        axial_sum = axial_sum + dist_arr[i][0]
        perp_sum = perp_sum + dist_arr[i][1]

    axial_avg = round(axial_sum / 6, round_n)

    perp_avg = round(perp_sum / 6, round_n)
    # print("init_axial_avg,",init_axial_avg)
    # print("init_perp_avg",init_perp_avg)
    print([round(-axial_avg * mm_per_pix, round_n), round(perp_avg * mm_per_pix, round_n)])
    return [axial_avg ,perp_avg]


def disp2strain(displacement):
    print("INITLA AXIL!!!!!!!!!!!!,", initial_axial)
    print("deformation", (displacement[0] - initial_axial))
    return [(displacement[0] - initial_axial) / initial_axial * 100,
            (displacement[1] - initial_perp) / initial_perp * 100]


def disp2strain_zero(displacement):
    return [0, 0]


def load_instron_data(TEST_NUMBER_):
    xlsxpath = "result_1201\\sample" + str(TEST_NUMBER_) + "\\sample" + str(TEST_NUMBER_) + ".xlsx"
    print("read file", xlsxpath)
    sgdata = pd.read_excel(xlsxpath)
    sgdata = sgdata.dropna()
    axialsg = sgdata[['axial_strain(%)']].to_numpy()
    perpsg = sgdata[['horizontal_strain(%)']].to_numpy()
    axialsg = sgdata[['axial_strain(%)']].to_numpy()
    perpsg = sgdata[['horizontal_strain(%)']].to_numpy()

    axialsg_s = axialsg[0:len(axialsg)].reshape(-1, 1)
    horizsg_s = perpsg[0:len(perpsg)].reshape(-1, 1)

    return axialsg, horizsg_s

def two_pdispstrain(centroid):
    axial1 = round(centroid[2][1]-centroid[0][1],round_n)
    axial2 = round(centroid[5][1]-centroid[3][1],round_n)
    axial3 = round(centroid[8][1]-centroid[6][1],round_n)
    axial = (axial1+axial2+axial3)/3
    print("The centorid values are")
    print(centroid)
    perp1 = round(centroid[2][0] - centroid[0][0], round_n)
    perp2 = round(centroid[5][0] - centroid[3][0], round_n)
    perp3 = round(centroid[5][0] - centroid[6][0], round_n)
    perp = (perp1+perp2+perp3)/3

    return [round(axial ,round_n), round(perp,round_n)]

def two_strain(displacement,init_axial_length):
    # init_axial_length = 111.33333333-49.8622449
    return [(displacement[0] - init_axial_length) / init_axial_length * 100,
            (displacement[1] - init_axial_length) / init_axial_length * 100]


def area_filter(ret, labels, stats, centroids,bin):
    sizearea = stats[:, 4]
    print(sizearea)
    cont = 0
    labelspeque = []
    areamin = 400
    areamax = 2500
    for i in range(0,ret):
        if sizearea[i] > areamin and sizearea[i]<areamax:
            labelspeque.append(i)

    print(labelspeque)
    # displaygplt(bin)

    return labelspeque


def template_match(img_rgb):
    # img_rgb = cv.imread("template\\0.png")
    img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)

    temp_path = "template\\template_rgb.png"
    template = cv.imread(temp_path, 0)
    # gray_template = cv.cvtColor(template,cv.COLOR_BGR2GRAY)
    w, h = template.shape[::-1]

    methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
                'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']

    img = img_gray.copy()
    method = eval(methods[1])


    # Apply template Matching
    res = cv.matchTemplate(img,template,method)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
    # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
    if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
        top_left = min_loc
    else:
        top_left = max_loc

    bottom_right = (top_left[0] + w, top_left[1] + h)
    center_point = [(bottom_right[0]+top_left[0])/2,(bottom_right[1]+top_left[1])/2]
    cv.rectangle(img,top_left, bottom_right, 255, 2)
    # plt.imshow(img,cmap = 'gray')
    # plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
    # plt.show()
    return center_point

def label_img(centroid):
    cx0 = centroid[0][0];
    cy0 = centroid[0][1];
    cx1 = centroid[1][0];
    cy1 = centroid[1][1];
    cx2 = centroid[2][0];
    cy2 = centroid[2][1];
    cx3 = centroid[3][0];
    cy3 = centroid[3][1];
    cx4 = centroid[4][0];
    cy4 = centroid[4][1];
    cx5 = centroid[5][0];
    cy5 = centroid[5][1];
    cx6 = centroid[6][0];
    cy6 = centroid[6][1];
    cx7 = centroid[7][0];
    cy7 = centroid[7][1];
    cx8 = centroid[8][0];
    cy8 = centroid[8][1];

    cent_tuple1 = [(cx0, cy0, cx0 + cy0), (cx1, cy1, cx1 + cy1), (cx2, cy2, cx2 + cy2)]
    cent_tuple2 = [(cx3, cy3, cx3 + cy3), (cx4, cy4, cx4 + cy4), (cx5, cy5, cx5 + cy5)]
    cent_tuple3 = [(cx6, cy6, cx6 + cy6), (cx7, cy7, cx7 + cy7), (cx8, cy8, cx8 + cy8)]

    cent1_sort = sorted(cent_tuple1, key=lambda entry1: entry1[2])
    cent2_sort = sorted(cent_tuple2, key=lambda entry2: entry2[2])
    cent3_sort = sorted(cent_tuple3, key=lambda entry3: entry3[2])

    print(cent1_sort[0][0])
    labeled = \
        [[cent1_sort[0][0], cent1_sort[0][1]],  # o    o    o          |    centroid[0]  centroid[3]   centroid[6]
         [cent1_sort[1][0], cent1_sort[1][1]],  # o    o    o          |    centroid[1]  centroid[4]   centroid[7]
         [cent1_sort[2][0], cent1_sort[2][1]],  # o    o    o          |    centroid[2]  centroid[5]   centroid[8]
         [cent2_sort[0][0], cent2_sort[0][1]],
         [cent2_sort[1][0], cent2_sort[1][1]],
         [cent2_sort[2][0], cent2_sort[2][1]],
         [cent3_sort[0][0], cent3_sort[0][1]],
         [cent3_sort[1][0], cent3_sort[1][1]],
         [cent3_sort[2][0], cent3_sort[2][1]]]

    return labeled


def resample_by_interpolation(signal, input_fs, output_fs):

    scale = output_fs / input_fs
    # calculate new length of sample
    n = round(len(signal) * scale)

    # use linear interpolation
    # endpoint keyword means than linspace doesn't go all the way to 1.0
    # If it did, there are some off-by-one errors
    # e.g. scale=2.0, [1,2,3] should go to [1,1.5,2,2.5,3,3]
    # but with endpoint=True, we get [1,1.4,1.8,2.2,2.6,3]
    # Both are OK, but since resampling will often involve
    # exact ratios (i.e. for 44100 to 22050 or vice versa)
    # using endpoint=False gets less noise in the resampled sound
    resampled_signal = np.interp(
        np.linspace(0.0, 1.0, n, endpoint=False),  # where to interpret
        np.linspace(0.0, 1.0, len(signal), endpoint=False),  # known positions
        signal,  # known data points
    )
    return resampled_signal


def calibrate_camera(img):
    data = np.load('calib.npz')
    h, w = img.shape[:2]
    mtx = data["mtx"]
    dist = data["dist"]
    rvecs = data["rvecs"]
    tvecs = data["tvecs"]
    newcameraMtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    dst = cv2.undistort(img, mtx, dist, None, newcameraMtx)

    x1, y1, w1, h1 = roi
    dst = dst[y1:y1 + h1, x1:x1 + w1]
    return dst

##TODO: 
## set TEST Number and calculate
## Cut frame is for pre-cut. do and find as experiment
## Final frame is for post-cut. Manually set
TEST_NUMBER =5
cut_frame = 0 #test1: 44        #test2:100      #test3:117 # test5: 55
final_frame = 50 #test1: 270      #test2: 329   #test3:350   #test5:550
TEST_NUMBER_=TEST_NUMBER


if __name__ == "__main__":
    #TODO: #File path 명시할 것 앞선 1,2 에서 설정한 file 을 설정할 것
    rst_img_dir =  "/home/kimyun/catkin_ws/src/SNU_SmartLAB/snu_idim_strainCam/result/" + "first/first.avi" #str(subject_name) + "/" + str(TEST_NUMBER) +".avi"
    cap = cv2.VideoCapture(rst_img_dir)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print("video information  total_frames ==" +str(total_frames))

    frame_count = 0
    disp_slist = []
    displist = []
    strain_list = []

    while (cap.isOpened()):
        #앞서 영상속 필요없는 부분 삭제 코드
        if frame_count<cut_frame:
            ret, frame = cap.read()
            frame_count+=1
            continue
        # 마지막 frame 도달시 나갈 것
        if frame_count==final_frame:
            break
        print("read frame at",frame_count)
        ret, frame_ = cap.read()

        #TODO: 이전에 체크보드를 통해 작성한 카메라 calibratiion matrix 이용 자르기
        frame = calibrate_camera(frame_)
        # displayplt(frame)
        img = cv2.rotate(frame, cv2.cv2.ROTATE_90_CLOCKWISE) #카메라 설정에 따라 변경할 것

        #TODO: 관심있는 프레임 시작시 시작해서 기존에 작성한 template에 맞춰서 ROI(Region of interest)를 구함
        if frame_count==cut_frame:
            center_x, center_y = template_match(img)
            w = 350;  h = 350; # 경험적으로 구함
            x = int(center_x-w/2); y = int(center_y-h/2)

        iterdilate = 1
        c_img = crop_image(img, x, w, y, h)
        # displayplt(c_img)
        # gray = cv2.cvtColor(c_img, cv2.COLOR_BGR2GRAY)
        #TODO: Filter out noise(이부분에서 iteration 수를 조정할 것)
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (4, 4))
        img_mask = cv.morphologyEx(c_img, cv.MORPH_DILATE, kernel, iterations=1) #for test10 it is 1 else 2
        img_mask = cv.morphologyEx(img_mask, cv.MORPH_ERODE, kernel, iterations=2)
        img_mask = cv.morphologyEx(img_mask, cv.MORPH_DILATE, kernel, iterations=2)
        # displayplt(c_img)


        gray = cv2.cvtColor(img_mask, cv2.COLOR_BGR2GRAY)

        # displaygplt(gray)
        # cv.imshow("gray", gray)
        # cv2.waitKey()
        # ret, bin = cv.threshold(gray, 120, 255, cv.THRESH_BINARY_INV)
        ret, bin = cv.threshold(gray, 80, 255, cv.THRESH_BINARY_INV) #TODO: 이부분에서 threshold 값들은 실험 초반에서 찾기

        # displaygplt(bin)
        cv2.imshow("bin",bin)
        cv2.waitKey(1)

        #서로 붙어 있는 요소들을 찾고 하나의 객체로 묶음
        nlabels, labels, stats, centroids = cv.connectedComponentsWithStats(bin,connectivity=4)

        #TODO: 각 객체들이 묶이고 나서 갯수가 부족한지 많은지 확인하여 조치하는 방법
        # 총 9개의 점이 나와야하는데 위 cv 함수의경우 0번을 제외한 1번부터 봐야하므로 nlabels 개수가 10개 이상
        centroid_ = []
        if nlabels >10: #
            print("filter is used")
            correct_idx = area_filter(nlabels, labels, stats, centroids,bin)
            print("correct_dix is", correct_idx)
            print("the area non sorted is",stats[:, 4])
            for it in range(0,len(correct_idx)):
                centroid_.append(centroids[correct_idx[it]])
            print("centroid sorted~!!!!!!,centroid",centroid_)

        elif nlabels ==10:
            centroid_ = centroids[1:10]
        else:
            print("/////////////////////////////////////")
            print("value error labels less than 10")
            print("/////////////////////////////////////")
            break

        print(frame_count)

        centroid__ = sorted(centroid_, key=lambda a_entry: a_entry[0])
        print("before sorting", centroid__)
        #TODO: 9개의 점들을 sorting을해서 각자의 위치에 맞는 번호를 매기는 과정 자세한 번호는 위 function을 직접 보면 그림 그려놨음
        centroid = label_img(centroid__)
        print("after sorting", centroid)

        # displacement = calculate_displacement(centroid)
        displacement = two_pdispstrain(centroid)
        dispsmall = calculate_displacement(centroid)
        if frame_count ==cut_frame: #첫 프레임의 경우 initial 값을 읽어옴
            strain = disp2strain_zero(displacement)
            # print(displacement)
            # strain = two_strain(displacement)
            initial_axial = displacement[0]
            initial_perp = displacement[1]
            print("THe displacement is!!!!!!!!")
            print(displacement)

            # cv2.imwrite("template.png", gray)
            displaygplt(bin)
        else:
            strain = two_strain(displacement,initial_axial)
            # print("the strain value is", strain)
            print("THe displacement is!!!!!!!!")
            print(displacement)
            print("the strain value is")
            print(strain)
            print("the dismplacement list is")
            print(displist)
            print("The initial value is")
            print(initial_axial)
            print("The strain list is")
            print(strain_list)
            # displaygplt(bin)

            if strain[0]>7:
                strain[0] = prev_ax
        print("The strain values are", strain)
        if frame_count==0:
            print("the centroid values at frame 1",centroid)
            # displaygplt(bin)
        if strain[0] < -0.5 or strain[0] > 7: # strain value 가 이상하게 나오는 경우 debug funciton
            print("the centroid values at frame 1", centroid)
            print("nlabels", nlabels)
            print("centroids",sorted(centroids, key=lambda a_entry: a_entry[0]))
            print("centroid", centroid)
            print("area", stats[:, 4])
            print("displacement", displacement[0])
            print("strain value", strain[0])
            print("displacement", displacement[0])
            print("strain value",strain)
            print("the initial axial",initial_axial)
            print("the calculated displacement",displacement[0])
            print("diffenrence", displacement[0]-initial_axial)
            print("strain in spot", (displacement[0]-initial_axial)/initial_axial*100)

        #TODO:  위에서 계산한 결과 값들을 변수에 모두 모음
        strain_list.append(strain)
        displist.append(displacement)
        disp_slist.append(dispsmall)
        prev_ax = strain[0]
        prev_perp = strain[1]
        frame_count += 1

    # 데이터 타입 변환
    displist1 = np.asarray(displist)
    dispsmall1 = np.asarray(disp_slist)
    strainx = np.asarray(strain_list)

    ## TODO: fps 설정 필요
    fps = 10
    spf = 1 / fps
    img_num = total_frames
    time_end = img_num * spf
    x = np.linspace(cut_frame/10, (final_frame)/10, final_frame-cut_frame)

    # x = np.linspace(0, frame_length * spf, frame_length - 1)
    dispsx, dispsy = np.hsplit(dispsmall1,[1])
    left, right = np.hsplit(strainx, [1])
    dispx, dispy = np.hsplit(displist1,[1])
    # axialsg_s, horizsg_s = load_instron_data(TEST_NUMBER)
    # axialsg=np.asarray(axialsg_s)
    # axialstrain_sg = axialsg[cut_frame:final_frame]
    #
    #
    # print(len(axialstrain_sg))
    # print(axialstrain_sg)
    # print("raw data is", axialsg)
    # x = np.linspace(cut_frame/10, final_frame/10, len(axialstrain_sg))
    # plt.plot(x,axialstrain_sg)
    # plt.show()

    filename = "/home/kimyun/catkin_ws/src/SNU_SmartLAB/snu_idim_strainCam/result/" + "first/first.xlsx"# + str(TEST_NUMBER_) + "\\sample"+str(TEST_NUMBER)+"_vision___.xlsx"
    dat = {

            'axial_strain(%)': np.ravel(left),
            'displacement': np.ravel(dispx),
            'dispsmall': np.ravel(dispsx)

        }
    df1 = pd.DataFrame(dat)
    writer = pd.ExcelWriter(filename, engine='xlsxwriter')
    df1.to_excel(writer, sheet_name='Sheet1')
    writer.close()