#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import pandas as pd
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
# sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../../snu_idim_common/imp")) )
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_SmartLAB/snu_idim_common/imp"%HOME_DIR)) )
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
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_transformStamped = TransformStamped()
        self.cmd_pose = Pose()
        self.vision_status = "vision protocol waiting"
        ### Topics to Subscribe ###
        self.flag_sub  = rospy.Subscriber("/R_001/vision_2d_flag", Int32, self.vision_flag, queue_size=1) # Trigger Topic
        self.imagewindowflag = False #False : image pass, True : image show for debugging
        self.specimensaveflag = False
        self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw", Image, self.vision)

        ### Topics to Publish ###
        self.pub1 = rospy.Publisher('cmd_moveit', Pose, queue_size=1) # Final target pose to be tracked

        print(self.vision_status)

        ### Trun on CNN specimen detecion
        os.system('python3 /home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/src/snu_vision_specimen_detection.py')

    def vision(self, data):
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

            location = '/home/syscon/Desktop/image/specimen.png'

        if self.imagewindowflag == False :
            pass
        elif self.imagewindowflag == True:
            cv2.imshow('robot endeffector image', self.cv_image)  #orignal image show
            cv2.waitKey(10)



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
        self.static_transformStamped.header.stamp = rospy.Time.now()
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
            pass

        # VISION [30002] : SEARCH SPECIMEN (CANNY -> CONTOUR)
        elif self.vision_protocol == TASK_JOG_DEVEL :
            location = '/home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/specimen_image/specimen.png'   
            cv2.imwrite(location, self.specimen_image)
            # os.system('python3 /home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/src/snu_vision_specimen_detection.py')
            while True:
                if os.path.exists('/home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/specimen_image/csv/info.csv'):
                    rospy.sleep(0.1)
                    specimen_info = pd.read_csv('/home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/specimen_image/csv/info.csv')
                    break
                else:
                    rospy.sleep(0.1)
                    # print('Csv file does not exist')

            specimen_np = specimen_info.to_numpy()
            size = np.shape(specimen_np)
            object_count = size[0]
            print 'Number of objects :' + str(object_count)
            for i in range(object_count):
                theta = specimen_np[i, 1]; px2mm_Row = specimen_np[i, 2]; px2mm_Col = specimen_np[i, 3]
                print(i+1, theta, px2mm_Row, px2mm_Col)
                for j in range(10):
                    self.tf_broadcaster(CAMERA_FRAME_PREFIX_, TEMP_PREFIX_ + str(i+1), 262.0, -px2mm_Row+22.0, -px2mm_Col-10.5, 0, np.pi/2, np.pi)
                    self.tf_broadcaster(TEMP_PREFIX_ + str(i+1), OBJECT_TARGET_PREFIX_ + str(i+1), 0.0, 0.0, 0.0, 0.0, 0.0, -theta+(90)*np.pi/180)
            
        os.remove('/home/syscon/catkin_ws/src/SNU_SmartLAB/snu_idim_cobot/specimen_image/csv/info.csv')
        self.vision_status = "vision processing complete"
        print(self.vision_status)

        ###############################################################################################################3
    
if __name__ == "__main__":
    twod_vision = snu_vision_2d()

    while not rospy.is_shutdown():
        pass

    print 'good bye!'
