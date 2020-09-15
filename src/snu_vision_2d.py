#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os, sys
import pandas as pd
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_IDIM_ASMR/common/imp"%HOME_DIR)) )
from IDIM_header import *
from IDIM_framework import *
from pytesseract import *

################  TO DO LIST  ################
# 1. organize parameter

################  TF NAME ################
CAMERA_FRAME_PREFIX_    = 'camera_link'
OBJECT_TARGET_PREFIX_   = 'specimen_table_'
TEMP_PREFIX_            = 'temp_'

class snu_vision_2d():
    def __init__(self, ros_node_name="snu_vision_2d"):
        
        rospy.init_node('ros_node_name', anonymous=True)
        self.bridge = CvBridge()      
        self.listener = tf.TransformListener()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = TransformStamped()
        self.cmd_pose = Pose()
        self.vision_status = "vision protocol waiting"
        ### Topics to Subscribe ###
        self.flag_sub  = rospy.Subscriber("/R_001/vision_2d_flag", Int32, self.vision_flag, queue_size=1) # Trigger Topic
        self.imagewindowflag = False #False : image pass, True : image show for debugging
        self.specimensaveflag = False
        self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw", Image, self.vision)
        self.image_count = 69
        self.time_rate =0
        ### Topics to Publish ###
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1) # Final target pose to be tracked

        print(self.vision_status)

    def vision(self, data):
        # pass
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.specimen_image = copy.deepcopy(self.cv_image) # move this when you draw something in the image and change imagewindowflag
        except CvBridgeError as e:
            print(e)

        if self.specimensaveflag == False :
            pass
        elif self.specimensaveflag == True :
            rowEnd=634  #616 for sindoh 622 for sondori
            colEnd=448  #402 for sindoh 416 for sondori
            rowStart=186 #241 for sindoh 233 for sondori
            colStart=0 #24 for sindoh 24 for sondori
            self.bgr_image = self.specimen_image[colStart:colEnd, rowStart:rowEnd]

            # location = '/home/syscon/Desktop/image/default' + str(self.image_count) + '.png'
            location = '/home/syscon/Desktop/image/specimen' + str(self.image_count) + '.png'
            # location = '/home/syscon/catkin_ws/src/SNU_IDIM_ASMR/specimen_image/specimen.png'   
            if self.time_rate%30 == 0:
                cv2.imwrite(location, self.bgr_image)
                cv2.imshow('cropped saving image', self.bgr_image)
                cv2.waitKey(1000)
                self.image_count += 1
            # rospy.sleep(1)    
            self.time_rate+=1


        if self.imagewindowflag == False :
            pass
        elif self.imagewindowflag == True:
            
            # rowEnd=634  #616 for sindoh 622 for sondori
            # colEnd=448  #402 for sindoh 416 for sondori
            # rowStart=186 #241 for sindoh 233 for sondori
            # colStart=0 #24 for sindoh 24 for sondori

            # location = '/home/syscon/Desktop/image/default' + str(self.image_count) + '.png'
            # location = '/home/syscon/Desktop/image/specimen' + str(self.image_count) + '.png'
            # location = '/home/syscon/catkin_ws/src/SNU_IDIM_ASMR/specimen_image/specimen.png'
            # self.bgr_image = self.specimen_image[colStart:colEnd, rowStart:rowEnd]

            # cv2.imwrite(location, self.specimen_image)
            # rospy.sleep(2)
            # self.image_count += 1
            
            # cv2.namedWindow('robot endeffector image', cv2.WINDOW_NORMAL)
            cv2.imshow('robot endeffector image', self.cv_image)  #orignal image show
            # cv2.imshow('robot endeffector image', self.bgr_image)   #cropped image show
            # cv2.iamge('robot unet image', self.unet_image)
            cv2.waitKey(5000)



    '''
        Translation: (trans, rot) -> geometry_msgs/Pose
    '''
    def update_cmd_pose(self, trans, rot):
        self.cmd_pose.position.x    = trans[0] # 보정 @(arm -> 측면)
        self.cmd_pose.position.y    = trans[1] # 보정 @(arm -> 정면)
        self.cmd_pose.position.z    = trans[2]
        self.cmd_pose.orientation.x = rot[0]
        self.cmd_pose.orientation.y = rot[1]
        self.cmd_pose.orientation.z = rot[2]
        self.cmd_pose.orientation.w = rot[3]


    def updateEulZYZ(self):
        q_w = self.cmd_pose.orientation.w
        q_x = self.cmd_pose.orientation.x
        q_y = self.cmd_pose.orientation.y
        q_z = self.cmd_pose.orientation.z
        t1 = math.atan2(q_x, q_y)
        t2 = math.atan2(q_z, q_w)
        z1 = t2 - t1 
        y1 = 2*math.acos(math.sqrt(q_w*q_w + q_z*q_z))
        z2 = t2 + t1  
        self.eulerZYZ = [RAD2DEG(z1), RAD2DEG(y1), RAD2DEG(z2)]
        print('The Euler angles are calculated:', self.eulerZYZ)

    '''
        Update ROS Parameters
    '''    
    def update_ros_param(self):
        pass

    '''
        TF : Static transform 
        TRANSLATION : X, Y, Z [mm] 
        ROTATION    : Z1, Y, Z2 [radian]
    ''' 
    def tf_broadcaster(self, PARENT_ID, CHILD_ID, TRANSLATION_X, TRANSLATION_Y, TRANSLATION_Z, ROTATION_Z1, ROTATION_Y, ROTATION_Z2):
        self.static_transformStamped.header.frame_id = PARENT_ID
        self.static_transformStamped.child_frame_id  = CHILD_ID
        self.static_transformStamped.transform.translation.x = MM2M(TRANSLATION_X)
        self.static_transformStamped.transform.translation.y = MM2M(TRANSLATION_Y)
        self.static_transformStamped.transform.translation.z = MM2M(TRANSLATION_Z)
        quat = tf.transformations.quaternion_from_euler(ROTATION_Z1, ROTATION_Y, ROTATION_Z2)
        self.static_transformStamped.transform.rotation.x = quat[0]
        self.static_transformStamped.transform.rotation.y = quat[1]
        self.static_transformStamped.transform.rotation.z = quat[2]
        self.static_transformStamped.transform.rotation.w = quat[3]
        self.broadcaster.sendTransform(self.static_transformStamped)

    def getorientation(self, pts):
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = pts[i,0]
            data_pts[i,1] = pts[i,1]
        mean = np.empty((0))
        mean, eigenvectors = cv2.PCACompute(data_pts, mean)
        cntr = (mean[0,0], mean[0,1])
        angle = math.atan2(eigenvectors[1,1], eigenvectors[1,0])

        return (angle, cntr)


    def vision_flag(self, msg):
        self.vision_status = "running"
        self.vision_protocol = msg.data
        # self.update_ros_param()
        print(self.vision_protocol)

        ########################################################################
        # VISION [30001] : SEARCH SPECIMEN (CANNY -> CONTOUR)
        if self.vision_protocol == VISION_2D_SEARCH_SPECIMEN :
            
            #Set Region of Interest
            rowEnd=639  #616 for sindoh 622 for sondori
            colEnd=480  #402 for sindoh 416 for sondori
            rowStart=156 #241 for sindoh 233 for sondori
            colStart=5 #24 for sindoh 24 for sondori

            obj_count=1

            try:
                bgr_temp = self.specimen_image
            except:
                print('CAMERA CONNECTION ERROR : check whether you are publishing 30001 or check camera connection')
                return -1

            hsv_temp = cv2.cvtColor(bgr_temp, cv2.COLOR_BGR2HSV)
            gray_temp = cv2.cvtColor(bgr_temp, cv2.COLOR_BGR2GRAY)
            # crop image
            gray = gray_temp[colStart:colEnd, rowStart:rowEnd]
            bgr = bgr_temp[colStart:colEnd, rowStart:rowEnd]
            hsv = hsv_temp[colStart:colEnd, rowStart:rowEnd]

            # print(hsv.shape)
            # print(hsv_temp.shape)

            # SONDORI BED FILTER
            # img = cv2.bilateralFilter(bgr, 9, 75, 75)
            # img = copy.deepcopy(bgr)
            # size_temp = np.shape(img)
            # for ii in range(size_temp[0]):
            #     for jj in range(size_temp[1]):
            #         # x = img[ii, jj, 0]; y = img[ii, jj, 1]; z = img[ii, jj, 2]
            #         # if abs(((z-1.0227*x)/19.4392)-1) < 0.8 and abs(((y-0.9905*x)/15.9725)-1) < 0.8:
            #             # img[ii,jj] = (255,255,255)
            #         if img[ii,jj,0] < 80 and img[ii,jj,1] < 80 and img[ii,jj,2] < 80:
            #             img[ii,jj] = (0,0,0)
            #         # if img[ii,jj,2] > 190:
            #             # img[ii,jj] = (255, 255, 255)

            # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                

            # # #FILTER1 HSV threshold
            # lower_bound = np.array([0, 0, 125])
            # upper_bound = np.array([255, 40, 200])
            # hsv_filter = cv2.inRange(hsv, lower_bound, upper_bound)

            # print(hsv)
            # hsv2gray = np.empty(shape=(colEnd-colStart, rowEnd-rowStart), dtype=np.uint8)
            # for ii in range(rowEnd-rowStart):
            #     for jj in range(colEnd-colStart):
            #         hsv2gray[jj, ii] = int(hsv[jj, ii, 0]/2 + hsv[jj, ii, 2]/2)
            #         # print(hsv2gray[jj,ii])
            # print(hsv2gray)
            # print(hsv2gray.shape)
            # print(gray)

            #Canny edge detection & Hough lines transform
            edges=cv2.Canny(bgr, 50, 200)
            _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            #image show for debugging
            # while True:
            # cv2.imshow('crop_image', bgr)
                # cv2.imshow('gray_image', gray)
                # cv2.imshow('hsv_image', hsv)
                # cv2.imshow('hsv_image_filter', hsv_filter)
                # cv2.imshow('hsvgray_image', hsv2gray)
            # cv2.imshow('color_filter_image', img)
            # cv2.imshow('Canny', edges)
                # cv2.imshow('Contour', contours)
            # cv2.waitKey(0)
            
            for ii in range(len(contours)):
                ptAccum=np.squeeze(contours[ii])
                print(len(ptAccum))
                

                # FILTER2 : the specimen edge should contain more than 300 points
                if (len(ptAccum) < 600 or len(ptAccum) > 1600) : 
                    print('bad search : points are too many or ')
                    # print(len(ptAccum))
                    continue
                # print(len(ptAccum))

                x_Max=np.argmax(ptAccum[:,0]) # x maximum coordinate index
                x_Min=np.argmin(ptAccum[:,0]) # x minimum coordinate index
                y_Max=np.argmax(ptAccum[:,1]) # y maximum coordinate index
                y_Min=np.argmin(ptAccum[:,1]) # y minimum coordinate index

                x_Max_Filter = 0; x_Min_Filter = 0; y_Max_Filter = 0; y_Min_Filter = 0; 
                x_Max_Accum = np.empty((0,2),int); x_Min_Accum = np.empty((0,2),int); y_Max_Accum = np.empty((0,2),int); y_Min_Accum = np.empty((0,2),int); 
                # print(ptAccum)
    
                #FILTER2
                for ii in range(len(ptAccum)):
                    # print(ptAccum[ii])
                    if ptAccum[ii, 0] >= ptAccum[x_Max,0] - 3:
                        x_Max_Filter += 1
                        x_Max_Accum = np.append(x_Max_Accum, np.array([ptAccum[ii]]), axis=0)
                    elif ptAccum[ii, 0] <= ptAccum[x_Min,0] + 3:
                        x_Min_Filter += 1
                        x_Min_Accum = np.append(x_Min_Accum, np.array([ptAccum[ii]]), axis=0)
                    if ptAccum [ii, 1] >= ptAccum[y_Max,1] - 4:
                        y_Max_Filter += 1
                        y_Max_Accum = np.append(y_Max_Accum, np.array([ptAccum[ii]]), axis=0)
                    elif ptAccum[ii, 1] <= ptAccum[y_Min,1] + 4:
                        y_Min_Filter += 1
                        y_Min_Accum = np.append(y_Min_Accum, np.array([ptAccum[ii]]), axis=0)
                
                # print(x_Max_Filter, x_Min_Filter, y_Max_Filter, y_Min_Filter)
                # print(x_Max_Accum, x_Min_Accum, y_Max_Accum, y_Min_Accum)

                if x_Max_Filter >= 40 and x_Min_Filter >= 40 and y_Max_Filter >= 40 and y_Min_Filter >= 40 :
                    perpen_Flag = True
                else :
                    perpen_Flag = False

                if perpen_Flag == False:
                    #find four rectnagular Vertices using maximum coordinate
                    x_Max_Vertice=[ptAccum[x_Max,0], ptAccum[x_Max,1]]
                    x_Min_Vertice=[ptAccum[x_Min,0], ptAccum[x_Min,1]]
                    y_Max_Vertice=[ptAccum[y_Max,0], ptAccum[y_Max,1]]
                    y_Min_Vertice=[ptAccum[y_Min,0], ptAccum[y_Min,1]]
                    vertice = [x_Min_Vertice, y_Max_Vertice, x_Max_Vertice, y_Min_Vertice]
                elif perpen_Flag == True:
                    # right_Top_Vertice=[np.average(x_Max_Accum[:,0]), np.max(x_Max_Accum[:,1])]
                    # left_Top_Vertice=[np.average(x_Min_Accum[:,0]), np.max(x_Min_Accum[:,1])]
                    # right_Bottom_Vertice=[np.average(x_Max_Accum[:,0]), np.min(x_Max_Accum[:,1])]
                    # left_Bottom_Vertice=[np.average(x_Min_Accum[:,0]), np.min(x_Min_Accum[:,1])]

                    right_Top_Vertice=[np.max(y_Max_Accum[:,0]), np.average(y_Max_Accum[:,1])]
                    left_Top_Vertice=[np.min(y_Max_Accum[:,0]), np.average(y_Max_Accum[:,1])]
                    right_Bottom_Vertice=[np.max(y_Min_Accum[:,0]), np.average(y_Min_Accum[:,1])]
                    left_Bottom_Vertice=[np.min(y_Min_Accum[:,0]), np.average(y_Min_Accum[:,1])]

                    vertice=[left_Top_Vertice, right_Top_Vertice, right_Bottom_Vertice, left_Bottom_Vertice]

                # FILTER3
                vertice_np = np.array(vertice)
                edge_Length=np.zeros([4,4])
                # print(vertice_np)
                for ii in range(4):
                    for jj in range(4):
                        edge_Length_Temp = np.linalg.norm(vertice_np[ii]-vertice_np[jj])
                        edge_Length[ii,jj] = edge_Length_Temp

                # print(edge_Length)
                # print(ptAccum[x_Max,0], ptAccum[x_Min,0], ptAccum[y_Max,1], ptAccum[y_Min,1])            

                # print(vertice)
                # orientation1 = float(x_Max_Vertice[1]-x_Min_Vertice[1])/float(x_Max_Vertice[0]-x_Min_Vertice[0])
                # orientation2 = float(y_Max_Vertice[1]-y_Min_Vertice[1])/float(y_Max_Vertice[0]-y_Min_Vertice[0])
                # orientation1 = float(vertice[0][1]-vertice[2][1])/float(vertice[0][0]-vertice[2][0])
                # orientation2 = float(vertice[1][1]-vertice[3][1])/float(vertice[1][0]-vertice[3][0])

                # orientation1 = float(vertice[0][0]-vertice[2][0])/float(vertice[0][1]-vertice[2][1])
                # orientation2 = float(vertice[1][0]-vertice[3][0])/float(vertice[1][1]-vertice[3][1])

                # orientation = (orientation1+orientation2)/2.0
                # theta = math.atan(orientation)

                #centroid : average of all coordinates
                # centroid=[np.average(ptAccum[:,0]), np.average(ptAccum[:,1])]
                # print('angle and centroid', theta, centroid)
                # print(ptAccum)
                (theta, centroid) = self.getorientation(ptAccum)
                print(theta, centroid)

                #plotting for debugging 
                cv2.circle(bgr, (int(centroid[0]), int(centroid[1])), 2, (0,0,255), 4)
                cv2.circle(bgr, (int(vertice[0][0]), int(vertice[0][1])), 1, (0,255,255), 2)
                cv2.circle(bgr, (int(vertice[1][0]), int(vertice[1][1])), 1, (0,255,255), 2)
                cv2.circle(bgr, (int(vertice[2][0]), int(vertice[2][1])), 1, (0,255,255), 2)
                cv2.circle(bgr, (int(vertice[3][0]), int(vertice[3][1])), 1, (0,255,255), 2)

                # while True : 
                cv2.imshow('image', bgr)
                cv2.waitKey(30)

                #Calibration 250mm / 551pixels -> sondori
                px2mm_Row=(centroid[0]+rowStart-320)*250.0/551.0
                px2mm_Col=(centroid[1]+colStart-240)*250.0/551.0

                if centroid is not None:
                    # self.tf_broadcaster(CAMERA_FRAME_PREFIX_, TEMP_PREFIX_ + str(obj_count), 262.0, -px2mm_Row+22.0, -px2mm_Col-10.5, 0, np.pi/2, np.pi)
                    self.tf_broadcaster(CAMERA_FRAME_PREFIX_, TEMP_PREFIX_ + str(obj_count), 262.0, -px2mm_Row+22.0, -px2mm_Col-10.5, 0, np.pi/2, np.pi)
                    self.tf_broadcaster(TEMP_PREFIX_ + str(obj_count), OBJECT_TARGET_PREFIX_ + str(obj_count), 0.0, 0.0, 0.0, 0.0, 0.0, -theta+(5.5+90)*np.pi/180)
                    obj_count = obj_count+1 


        # VISION [30002] : SEARCH SPECIMEN (CANNY -> CONTOUR)
        elif self.vision_protocol == TASK_JOG_DEVEL :
            location = '/home/syscon/catkin_ws/src/SNU_IDIM_ASMR/specimen_image/specimen.png'   
            cv2.imwrite(location, self.specimen_image)
            rospy.sleep(1)    
            os.system('python3 /home/syscon/test2.py')
            rospy.sleep(1)
            specimen_info = pd.read_csv('/home/syscon/catkin_ws/src/SNU_IDIM_ASMR/specimen_image/specimen.csv')
            specimen_np = specimen_info.to_numpy()
            # print(specimen_np)
            size = np.shape(specimen_np)
            object_count = size[0]
            print 'Number of objects :' + str(object_count)

            for i in range(object_count):
                theta = specimen_np[i, 1]; px2mm_Row = specimen_np[i, 2]; px2mm_Col = specimen_np[i, 3]
                print(i+1, theta, px2mm_Row, px2mm_Col)
                # self.tf_broadcaster('camera_color_frame', TEMP_PREFIX_ + str(i+1), 262.0, -px2mm_Row, px2mm_Col, 0, 0, 0)
                self.tf_broadcaster(CAMERA_FRAME_PREFIX_, TEMP_PREFIX_ + str(i+1), 262.0, -px2mm_Row+22.0, -px2mm_Col-10.5, 0, np.pi/2, np.pi)
                self.tf_broadcaster(TEMP_PREFIX_ + str(i+1), OBJECT_TARGET_PREFIX_ + str(i+1), 0.0, 0.0, 0.0, 0.0, 0.0, -theta+(90)*np.pi/180)

        self.vision_status = "vision processing complete"
        print(self.vision_status)

        ###############################################################################################################3
    
if __name__ == "__main__":
    twod_vision = snu_vision_2d()

    while not rospy.is_shutdown():
        pass

    print 'good bye!'
