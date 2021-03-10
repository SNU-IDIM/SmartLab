#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import json
from threading import Thread
from time import sleep
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../../snu_idim_common/imp")) )
from IDIM_header import *
from IDIM_framework import *


class DeviceClass_Cobot():
    def __init__(self, device_name="cobot"):
        rospy.init_node(device_name, anonymous=True)
        
        rospy.Subscriber("cobot/command", String, self.pnp_cb, queue_size=1)
        rospy.Subscriber("dsr/state", RobotState, self.dsr_state_cb, queue_size=1)

        self.cobot_status_pub = rospy.Publisher("cobot/status", String, queue_size=1)
        self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw",Image,self.vision_cb)
        self.vision_pub = rospy.Publisher("vision_2d_flag",Int32, queue_size=1)
        self.gripper_pub = rospy.Publisher("PC_to_GRIPPER", String, queue_size=1)

        self.status_overwrite = None

        self.status = dict()
        self.status['device_type'] = 'Collaborative_Robot'
        self.status['device_name'] = device_name
        self.status['status'] = None
        self.status['current_work'] = None
        self.status['recent_work'] = None

        self.status['posj'] = None
        self.status['posx'] = None
        self.status['torque'] = None
        self.status['force'] = None

        self.status['gripper_type'] = None
        self.status['gripper_angle'] = -45
        self.status['gripper_state_mech'] = 'Open'
        self.status['gripper_state_suction'] = 'On'
        
        self.status['compressor'] = 'Off'
        self.status['toolchanger'] = 'Attach'
        self.status['universal_jig_x'] = 'Open'
        self.status['universal_jig_y'] = 'Open'

        self.enum_robot_state = dict()
        self.enum_robot_state['0']  = 'Initializing'
        self.enum_robot_state['1']  = 'Standby'
        self.enum_robot_state['2']  = 'Moving'
        self.enum_robot_state['3']  = 'Safe Off'
        self.enum_robot_state['4']  = 'Teaching'
        self.enum_robot_state['5']  = 'Safe Stop'
        self.enum_robot_state['6']  = 'Emergency Stop'
        self.enum_robot_state['7']  = 'Homming'
        self.enum_robot_state['8']  = 'Recovery'
        self.enum_robot_state['9']  = 'Safe Stop 2'
        self.enum_robot_state['10'] = 'Safe Off 2'

        self.target_pose = Pose()
        self.drl_pose = Q_TOP_PLATE
        self.eulerZYZ = np.zeros(3)
        self.cmd_protocol = ACTION_HOME

        self.toolforce_max      = 0.0
        self.toolforce_max_flag = False
        
        self.orient_truth       = 0.0
        self.orient_roll        = 0.0
        self.orient_pitch       = 0.0
        self.orient_yaw         = 0.0
        self.orient_calib_flag  = False
        self.orient_calib_exec  = False

        self.offset_x           = 0.0
        self.offset_y           = -0.12
        self.offset_z           = 0.2
        self.robvelj            = 30
        self.robaccj            = 30
        self.robvelx            = 50
        self.robaccx            = 50
        
        ##opencv drawing related
        self.imagewindowflag =0
        self.bridge = CvBridge()
        
        set_robot_mode(ROBOT_MODE_AUTONOMOUS);    rospy.sleep(1)

        # self.thread_1 = Thread(target=self.publishStatus)
        # self.thread_1.start()
        

    def publishStatus(self):
        msg_json = json.dumps(self.status)
        self.cobot_status_pub.publish(msg_json)
        # print("\n==============================================================")
        # print("[DEBUG] device_type: {}".format(self.status['device_type']))
        # print("[DEBUG] device_name: {}".format(self.status['device_name']))
        # print("[DEBUG] status: {}".format(self.status['status']))
        # print("[DEBUG] current_work: {}".format(self.status['current_work']))
        # print("[DEBUG] recent_work: {}".format(self.status['recent_work']))
        # print("[DEBUG] posj: {}".format(self.status['posj']))
        # print("[DEBUG] posx: {}".format(self.status['posx']))
        # print("[DEBUG] force: {}".format(self.status['force']))
        # print("[DEBUG] torque: {}".format(self.status['torque']))
        # print("[DEBUG] compressor: {}".format(self.status['compressor']))
        # print("[DEBUG] toolchanger: {}".format(self.status['toolchanger']))
        # print("[DEBUG] universal_jig_x: {}".format(self.status['universal_jig_x']))
        # print("[DEBUG] universal_jig_y: {}".format(self.status['universal_jig_y']))
        # print("[DEBUG] gripper_type: {}".format(self.status['gripper_type']))
        # print("[DEBUG] gripper_state_suction: {}".format(self.status['gripper_state_suction']))
        # print("[DEBUG] gripper_state_mech: {}".format(self.status['gripper_state_mech']))




    '''
        dsr_state_cb: "~/dsr/state" topic callback function (update dsr_status)
    '''
    def dsr_state_cb(self, msg):
        try:
            self.status['status'] = self.enum_robot_state[str(msg.robot_state)]
            self.status['status'] = "Moving" if self.status_overwrite == "Moving" else self.status['status']

            self.status['posx'] = msg.current_posx
            self.status['posj'] = msg.current_posj
            self.status['force'] = msg.actual_ett
            self.status['torque'] = msg.actual_ejt

            io_ctrlbox_status = msg.ctrlbox_digital_output
            self.status['compressor'] = 'On' if io_ctrlbox_status[0] == 1 else 'Off'
            self.status['toolchanger'] = 'Detach' if io_ctrlbox_status[1] == 1 else 'Attach'
            self.status['universal_jig_x'] = 'Close' if io_ctrlbox_status[4] == 1 else 'Open'
            self.status['universal_jig_y'] = 'Close' if io_ctrlbox_status[5] == 1 else 'Open'
            self.status['gripper_state_suction'] = 'On' if io_ctrlbox_status[7] == 1 else 'Off'

            io_flange_status = msg.flange_digital_output
            self.status['gripper_state_mech'] = 'Close' if io_flange_status[1] == 1 else 'Open'

            ## Initialize Maximum tool force
            if not self.toolforce_max_flag:
                self.toolforce_max = self.status['force'][2]
                self.toolforce_max_flag = True
            
            ## Capture Maximum tool force
            if self.toolforce_max < self.status['force'][2]:
                self.toolforce_max = self.status['force'][2]
                # print(self.toolforce_max)
        except:
            pass
    
    
    def calcRelMove(self, waypoint, eef_flag):
        # for i in range(len(waypoint)):
        #     waypoint[i] += self.drl_pose[i]
        if eef_flag == True:
            eef_angle = -DEG2RAD(self.status['gripper_angle'])
            print(self.status['gripper_angle'])
            if abs(eef_angle) > EPSILON:
                dx = deepcopy(waypoint[0])
                dy = deepcopy(waypoint[1])
                waypoint[0] = dx * math.cos(eef_angle) + dy * math.sin(eef_angle)
                waypoint[1] = dx * math.sin(eef_angle) + dy * math.cos(eef_angle)
                print(waypoint)
                return waypoint
            else:
                return waypoint
        else:
            return waypoint
    

    '''
        vision_cb: OpenCV image visualization
    '''
    def vision_cb(self, data):
        # pass
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.draw_image = copy.deepcopy(self.cv_image)
            self.specimen_image = copy.deepcopy(self.draw_image) # move this when you draw something in the image and change imagewindowflag
        except CvBridgeError as e:
            print(e)
        # if self.imagewindowflag ==0:
        #     cv2.namedWindow('robot endeffector image', cv2.WINDOW_NORMAL)
        #     cv2.imshow('robot endeffector image', self.cv_image)
        #     cv2.waitKey(1)
        # elif self.imagewindowflag ==1:
        #     cv2.namedWindow('robot endeffector image', cv2.WINDOW_NORMAL)
        #     cv2.imshow('robot endeffector image', self.draw_image)
        #     cv2.waitKey(1)
        
    '''
        Translation: (trans, rot) -> geometry_msgs/Pose
    '''
    def update_target_pose(self, trans, rot):
        self.target_pose.position.x    = M2MM(trans[0]) # 보정 @(arm -> 측면)
        self.target_pose.position.y    = M2MM(trans[1]) # 보정 @(arm -> 정면)
        self.target_pose.position.z    = M2MM(trans[2])
        self.target_pose.orientation.x = rot[0]
        self.target_pose.orientation.y = rot[1]
        self.target_pose.orientation.z = rot[2]
        self.target_pose.orientation.w = rot[3]

    '''
        updateEulZYZ: Calculate ZYZ rotation to feed 'movel' function for Doosan-robot
    '''
    def updateEulZYZ(self):
        q_w = self.target_pose.orientation.w
        q_x = self.target_pose.orientation.x
        q_y = self.target_pose.orientation.y
        q_z = self.target_pose.orientation.z
        t1 = math.atan2(q_x, q_y)
        t2 = math.atan2(q_z, q_w)
        z1 = t2 - t1 
        y1 = 2*math.acos(math.sqrt(q_w*q_w + q_z*q_z))
        z2 = t2 + t1  
        self.eulerZYZ = [RAD2DEG(z1), RAD2DEG(y1), RAD2DEG(z2)]
        # print('The Euler angles are calculated:', self.eulerZYZ)


    '''
        search_ar_target: lookupTransform to get AR_Target (calculated from AR_Marker)
            @ input 1: int ar_tag_number (ex - 0, 1, 2, 3, ...)
    '''
    def ARsearchFromBase(self, ar_tag_number):
        target_frame_name = 'ar_target_' + str(ar_tag_number)
        reference_frame_name = 'base_0'
        # print "Searching AR tag ..."
        # print("Target frame: "    + target_frame_name)
        # print("Reference frame: " + reference_frame_name)
        listener = tf.TransformListener()
        try:
            # print "Trying to search the target: %s ..."%target_frame_name
            listener = tf.TransformListener()
            listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(1.0))
            (trans,rot) = listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))

            self.target_pose.position.x    = M2MM(trans[0])# + self.offset_x) # 보정 @(arm -> 측면)
            self.target_pose.position.y    = M2MM(trans[1])# + self.offset_y) # 보정 @(arm -> 정면)
            self.target_pose.position.z    = M2MM(trans[2])# + self.offset_z)
            self.target_pose.orientation.x = rot[0]
            self.target_pose.orientation.y = rot[1]
            self.target_pose.orientation.z = rot[2]
            self.target_pose.orientation.w = rot[3]
            self.updateEulZYZ()
            pos = self.target_pose.position;   ori = self.eulerZYZ
            self.drl_pose = deepcopy(posx(pos.x, pos.y, pos.z ,ori[0], ori[1], ori[2]))
            return True
        except (Exception):
            print "[ERROR]: The Target(TF) is not Detected !!!"
            return False


    def ARsearchFromEEF(self, ar_tag_number):
        target_frame_name = 'ar_target_' + str(ar_tag_number)
        reference_frame_name = 'link6'
        # print "Searching AR tag ..."
        # print("Target frame: "    + target_frame_name)
        # print("Reference frame: " + reference_frame_name)
        listener = tf.TransformListener()
        try:
            # print "Trying to search the target: %s ..."%target_frame_name
            listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(1.0))
            (trans,rot) = listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
            print("[DEBUG] Trans: {}, Rot: {}".format(trans, rot))

            self.target_pose.position.x    = M2MM(trans[0])# + self.offset_x) # 보정 @(arm -> 측면)
            self.target_pose.position.y    = M2MM(trans[1])#f + self.offset_y) # 보정 @(arm -> 정면)
            self.target_pose.position.z    = M2MM(trans[2])# + self.offset_z)
            self.target_pose.orientation.x = rot[0]
            self.target_pose.orientation.y = rot[1]
            self.target_pose.orientation.z = rot[2]
            self.target_pose.orientation.w = rot[3]
            return True
        except (Exception):
            print "[ERROR]: The Target(TF) is not Detected !!!"
            return False
            
    def ARcheckFlipped(self, ar_tag_number):
        pass
        self.ARgetOrientRPY(ar_tag_number)
        if self.orient_calib_flag == True:
            print("flip check start !!!")
            self.orient_truth = self.orient_yaw
            self.orient_calib_flag = False
        else:
            angle_error = RAD2DEG(self.orient_truth - self.orient_yaw)
            tol = 20
            if angle_error < -(180.0+tol):
                angle_error += 360.0
            if angle_error > (180.0+tol):
                angle_error -= 360.0

            print(angle_error)
            
            # math.sin(DEG2RAD(angle_error)) < 0 + tol and math.sin(DEG2RAD(angle_error)) > 0 - tol ## 0 deg
            # math.sin(DEG2RAD(angle_error)) > 1 - tol ## 90 deg
            # math.sin(DEG2RAD(angle_error)) < -1 + tol
            # math.sin(DEG2RAD(angle_error)) < 0 + tol and math.sin(DEG2RAD(angle_error)) > 0 - tol

            if abs(angle_error) > 0 + tol and self.orient_calib_exec == False:
                if angle_error < 90 + tol and angle_error > 90 - tol:
                    print("AR Flipped: 90 deg, (error: {})".format(angle_error))
                    dx = -rospy.get_param('/R_001/snu_object_tracker/offset_from_target/y')
                    dy = -rospy.get_param('/R_001/snu_object_tracker/offset_from_target/x')
                    rz = 90
                elif angle_error < -90 + tol and angle_error > -90 - tol:
                    print("AR Flipped: -90 deg, (error: {})".format(angle_error))
                    dx = rospy.get_param('/R_001/snu_object_tracker/offset_from_target/y')
                    dy = rospy.get_param('/R_001/snu_object_tracker/offset_from_target/x')
                    rz = -90
                    # dx = -rospy.get_param('/R_001/snu_object_tracker/offset_from_target/x')
                    # dy = -rospy.get_param('/R_001/snu_object_tracker/offset_from_target/y')
                    # rz = 90
                elif angle_error < 180 + tol and angle_error > 180 - tol:
                    print("AR Flipped: 180 deg, (error: {})".format(angle_error))
                    dx = -rospy.get_param('/R_001/snu_object_tracker/offset_from_target/x')
                    dy = -rospy.get_param('/R_001/snu_object_tracker/offset_from_target/y')
                    rz = 0
                else:
                    print("AR Flipped: not flipped, (error: {})".format(angle_error))
                    dx = rospy.get_param('/R_001/snu_object_tracker/offset_from_target/x')
                    dy = rospy.get_param('/R_001/snu_object_tracker/offset_from_target/y')
                    rz = 180

                # dx = rospy.get_param('/R_001/snu_object_tracker/offset_from_target/x')
                # dy = rospy.get_param('/R_001/snu_object_tracker/offset_from_target/y')
                # rz = 180
                rospy.set_param('/R_001/snu_object_tracker/offset_from_target/x',  dx)
                rospy.set_param('/R_001/snu_object_tracker/offset_from_target/y',  dy)
                rospy.set_param('/R_001/snu_object_tracker/offset_from_target/rz', rz)
                rospy.sleep(3.0)
                self.ARsearchFromEEF(ar_tag_number)
                self.orient_calib_exec = True

    def ARgetOrientRPY(self, ar_tag_number):
        if self.ARsearchFromEEF(ar_tag_number):
            orientation = self.target_pose.orientation
            self.orient_roll  = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[0]
            self.orient_pitch = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[1]
            self.orient_yaw   = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
            # print("Roll: {}".format(RAD2DEG(self.orient_roll)))
            # print("Pitch: {}".format(RAD2DEG(self.orient_pitch)))
            # print("Yaw: {}".format(RAD2DEG(self.orient_yaw)))
            
            
    def ARalignMove(self, ar_tag_number):
        # self.ARcheckFlipped(ar_tag_number)
        self.updateEulZYZ()
        pos = self.target_pose.position;   ori = self.eulerZYZ
        self.drl_pose = deepcopy(posx(pos.x, pos.y, pos.z ,ori[0], ori[1], ori[2]))
        movel(self.drl_pose, ref=DR_TOOL, mod=DR_MV_MOD_REL)


    def ARsetReference(self, ar_tag_number, iter=1):
        self.ARsearchFromEEF(ar_tag_number)
        self.orient_calib_flag = True
        for i in range(iter):
            if self.ARsearchFromEEF(ar_tag_number) == True:
                self.ARalignMove(ar_tag_number)
                rospy.sleep(2.0)
        self.orient_calib_exec = False


    '''
        ARupdateParam: Updating parameters for target pose w.r.t. AR_Marker
            @ input 1: double dx [m]
            @ input 2: double dy [m]
            @ input 3: double dz [m]
    '''
    def ARupdateParam(self, dx, dy, dz, rx=DEFAULT_OFFSET_FROM_TARGET_RX, ry=DEFAULT_OFFSET_FROM_TARGET_RY, rz=DEFAULT_OFFSET_FROM_TARGET_RZ):
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/x', dx)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/y', dy)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/z', dz)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/rx', rx)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/ry', ry)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/rz', rz)
        rospy.sleep(2)

    def getColorSensor(self, number):
        if isinstance(number, int) and number >= 1 and number <= 8:
            movej(Q_COLOR_SENSOR_TRAY_RETRACT)
            if number == 1:
                movel(P_COLOR_SENSOR_TRAY_LEFT)
                movel(P_COLOR_SENSOR_1)
            if number == 2:
                movel(P_COLOR_SENSOR_TRAY_LEFT)
                movel(P_COLOR_SENSOR_2)
            if number == 3:
                movel(P_COLOR_SENSOR_TRAY_LEFT)
                movel(P_COLOR_SENSOR_3)
            if number == 4:
                movel(P_COLOR_SENSOR_TRAY_LEFT)
                movel(P_COLOR_SENSOR_4)
            if number == 5:
                movel(P_COLOR_SENSOR_TRAY_RIGHT)
                movel(P_COLOR_SENSOR_5)
            if number == 6:
                movel(P_COLOR_SENSOR_TRAY_RIGHT)
                movel(P_COLOR_SENSOR_6)
            if number == 7:
                movel(P_COLOR_SENSOR_TRAY_RIGHT)
                movel(P_COLOR_SENSOR_7)
            if number == 8:
                movel(P_COLOR_SENSOR_TRAY_RIGHT)
                movel(P_COLOR_SENSOR_8)
            self.movel_z(5)
            self.suction_cup_on()
            self.movel_z(-5)
            movej(Q_COLOR_SENSOR_TRAY_RETRACT)


    '''
        Doosan-robot Relative Move (translation in x, y, z [mm])
            @ input 1: double distance [mm]
            @ input 2: intArray velx = [50, 10] [mm/s, mm/s]
            @ input 3: intArray accx = [50, 10] [mm/s^2, mm/s^2]
    '''
    def movel_x(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_TOOL, mod=DR_MV_MOD_REL): # distance [mm]
        movel(posx(distance, 0, 0, 0, 0, 0), vel=velx, acc=accx, ref=ref, mod=mod)
    def movel_y(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_TOOL, mod=DR_MV_MOD_REL): # distance [mm]
        movel(posx(0, distance, 0, 0, 0, 0), vel=velx, acc=accx, ref=ref, mod=mod)
    def movel_z(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_TOOL, mod=DR_MV_MOD_REL): # distance [mm]
        movel(posx(0, 0, distance, 0, 0, 0), vel=velx, acc=accx, ref=ref, mod=mod)
    def movel_xyz(self, x, y, z, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_TOOL, mod=DR_MV_MOD_REL): # distance [mm]
        movel(posx(x, y, z, 0, 0, 0), vel=velx, acc=accx, ref=ref, mod=mod)
    def movel_xyzjoint(self,x,y,z,jz1,jy,jz2,velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_TOOL, mod=DR_MV_MOD_REL): # distance [mm], angle [degree]
        movel(posx(x, y, z, jz1, jy, jz2), vel=velx, acc=accx, ref=ref, mod=mod)

    # rotate z refer to base
    def rotate_z(self, jz, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL):
        movel(posx(0,0,0,0,0,jz), vel=velx, acc=accx, ref=ref, mod=mod)

    # move x, y, z refer to base
    def movel_y_base(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL): # distance [mm]
        movel(posx(0, distance, 0, 0, 0, 0), vel=velx, acc=accx, ref=ref, mod=mod)
    def movel_x_base(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL): # distance [mm]
        movel(posx(distance, 0, 0, 0, 0, 0), vel=velx, acc=accx, ref=ref, mod=mod)
    def movel_z_base(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL): # distance [mm]
        movel(posx(0, 0, distance, 0, 0, 0), vel=velx, acc=accx, ref=ref, mod=mod)


    def move_lack_pick(self):
        self.gripper_open()
        movel(posx(0, -23, -23, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)
        self.gripper_close()
        movel(posx(0, -2, 2, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(0, 40, 40, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)


    def move_lack_place(self):
        self.gripper_close()
        movel(posx(0, -20, -20, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)
        self.gripper_open()
        movel(posx(0, -40, 40, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)


    def specimen_shaking(self):
        task_compliance_ctrl([10000, 10000, 10000, 1500, 1500, 500])
        self.movel_z(2);      self.gripper_close();  self.movel_z(-2)
        self.rotate_z(10);    self.rotate_z(-20);    self.rotate_z(10)

        self.gripper_open();  self.movel_z(-2);      self.movel_y(120)
        self.movel_z(4);      self.gripper_close();  self.movel_z(-2)
        self.rotate_z(-10);   self.rotate_z(20);     self.rotate_z(-10)

        self.gripper_open();  self.movel_y(-60)
        self.movel_z(2);      self.gripper_close();  self.movel_z(-2)
        self.rotate_z(-10);   self.rotate_z(20);     self.rotate_z(-10)

        self.movel_x(-20)


    '''
        setVelAcc: Set Doosan-robot Velocity(joint, task), Acceleration(joint, task)
            @ input 1: int velj

    '''
    def setVelAcc(self, velj=DSR_DEFAULT_VELJ, accj=DSR_DEFAULT_ACCJ, velx=DSR_DEFAULT_VELX, accx=DSR_DEFAULT_ACCX):
        set_velj(velj)
        set_accj(accj)
        set_velx(velx[0], velx[1])
        set_accx(accx[0], accx[1])


    '''
        DSR Controller I/O Functions
            1. Compressor on/off
            2. Tool Changer attach/detach
            3. Suction Cup Gripper on/off
            4. Universal Jig X-axis open/close
    '''
    def IO_init(self):
        for i in range(1, 16+1):
            set_digital_output(i, 0)
        rospy.sleep(0.5)
        
    def compressor_on(self):
        pin = ACTION_IO_COMPRESSOR
        if get_digital_output(pin) == 0:
            set_digital_output(pin,1)
    def compressor_off(self):
        pin = ACTION_IO_COMPRESSOR
        if get_digital_output(pin) == 1:
            set_digital_output(pin,0)
    def toolchanger_attach(self):
        pin = ACTION_IO_TOOLCHANGER
        if get_digital_output(pin) == 1:
            set_digital_output(pin,0)
    def toolchanger_detach(self):
        pin = ACTION_IO_TOOLCHANGER
        if get_digital_output(pin) == 0:
            set_digital_output(pin,1)
    def jig_x_close(self):
        pin = ACTION_IO_JIG_X
        if get_digital_output(pin) == 0:
            set_digital_output(pin,1)
    def jig_x_open(self):
        pin = ACTION_IO_JIG_X
        if get_digital_output(pin) == 1:
            set_digital_output(pin,0)
    def jig_y_close(self):
        pin = ACTION_IO_JIG_Y
        if get_digital_output(pin) == 0:
            set_digital_output(pin,1)
    def jig_y_open(self):
        pin = ACTION_IO_JIG_Y
        if get_digital_output(pin) == 1:
            set_digital_output(pin,0)
    def suction_cup_on(self):
        pin = ACTION_IO_SUCTIONCUP
        if get_digital_output(pin) == 0:
            set_digital_output(pin,1)
    def suction_cup_off(self):
        pin = ACTION_IO_SUCTIONCUP
        if get_digital_output(pin) == 1:
            set_digital_output(pin,0)
    def rotate_on(self):
        pin = 9
        if get_digital_output(pin) == 0:
            set_digital_output(pin,1)
    def rotate_off(self):
        pin = 9
        if get_digital_output(pin) == 1:
            set_digital_output(pin,0)

    '''
        DSR Flange I/O Functions
            1. Parallel Gripper open/close
    '''
    def gripper_open(self):
        set_tool_digital_output(2, 0)
        rospy.sleep(0.1)
        if get_tool_digital_output(1) == 0:
            set_tool_digital_output(1, 1)
            
    def gripper_close(self):
        set_tool_digital_output(1, 0)
        rospy.sleep(0.1)
        if get_tool_digital_output(2) == 0:
            set_tool_digital_output(2, 1)

    
    def searchARTagFromRight(self):
        self.setVelAcc(50, 50, [100,50], [100,50])
        movej(Q_SEARCH_3DP_RIGHT)
        for i in range(8):
            tag_id = i + 1
            print(self.ARsearchFromEEF(tag_id))
            if self.ARsearchFromEEF(tag_id) == True:
                movej(Q_TOP_PLATE)
                print("[DEBUG] AR Tag: {}".format(tag_id))
                return tag_id
        movej(Q_TOP_PLATE)
        print("[DEBUG] No AR Tag found")
        return None

    def searchARTagFromRobot(self):
        self.setVelAcc(50, 50, [100,50], [100,50])
        movej(Q_TOP_PLATE)
        for i in range(8):
            tag_id = i + 1
            print(self.ARsearchFromEEF(tag_id))
            if self.ARsearchFromEEF(tag_id) == True:
                print("[DEBUG] AR Tag: {}".format(tag_id))
                return tag_id
        print("[DEBUG] No AR Tag found")
        return None


    def moveBedFromStageToRobot(self, tag_id_bed, tag_id_stage=None):
        self.setVelAcc(50, 50, [100,50], [100,50])
        self.jig_x_open();  self.jig_y_open();  rospy.sleep(1)
        movej(Q_SEARCH_3DP_RIGHT)

        self.ARupdateParam(-0.12, 0.0, 0.25, rx=180.0, ry=0.0, rz=180.0); rospy.sleep(1)
        if self.ARsearchFromEEF(tag_id_bed) == True:
            self.ARsetReference(tag_id_bed, 5)
            self.status['gripper_angle'] = 225

            waypoint_1 = self.calcRelMove([50, 0, -100, 0, 0, self.status['gripper_angle']], False)
            waypoint_2 = self.calcRelMove([-215, 0, 172, 0, 0, 0], True)
            waypoint_3 = self.calcRelMove([0, 0, 43, 0, 0, 0], True)
            waypoint_4 = self.calcRelMove([0, 0, -80, 0, 0, 0], True)
            waypoint_5 = self.calcRelMove([300, 0, 0, 0, 0, 0], True)
            waypoint_6 = deepcopy(P_UNIVERSALJIG_3DP_BED);   waypoint_6[2] += 100
            waypoint_7 = deepcopy(waypoint_6);               waypoint_7[2] -= 140

            movel(waypoint_1, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            movel(waypoint_2, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            movel(waypoint_3, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            self.suction_cup_on()
            movel(waypoint_4, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            movel(waypoint_5, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            movel(waypoint_6)
            movel(waypoint_7)
            self.suction_cup_off();  rospy.sleep(1)
            self.jig_x_close();  self.jig_y_close();  rospy.sleep(1)
            movel(waypoint_6)
            movej(Q_TOP_PLATE)
            self.ARupdateParam(-0.12, 0.0, 0.25, rx=180.0, ry=0.0, rz=180.0)


    def moveBedFromRobotToStage(self, tag_id_bed, tag_id_stage):
        self.setVelAcc(50, 50, [100,100], [100,100])
        self.jig_x_close();  self.jig_y_close();  rospy.sleep(1)
        movej(Q_TOP_PLATE)

        if self.ARsearchFromEEF(tag_id_bed) == True: ## when AR tag is detected, execute the following codes
            movej(Q_SEARCH_3DP_RIGHT)
            if self.ARsearchFromEEF(tag_id_stage) == True:
                self.ARupdateParam(-0.12, 0.0, 0.25, rx=180.0, ry=0.0, rz=180.0); rospy.sleep(1)
                movej(Q_TOP_PLATE)
            else:
                movej(Q_HOME)
                return -1

            waypoint_0 = deepcopy(Q_SEARCH_3DP_PLATE)
            movej(waypoint_0)
            self.movel_z(105)
            self.suction_cup_on();  rospy.sleep(1)
            self.jig_x_open();  self.jig_y_open(); rospy.sleep(1)
            self.movel_z(-105)
        
            movej(Q_SEARCH_3DP_RIGHT)

            if self.ARsearchFromEEF(tag_id_stage) == True:
                self.ARsetReference(tag_id_stage, 4)
                self.status['gripper_angle'] = 223
                waypoint_1 = self.calcRelMove([0, 90, 0, 0, 0, self.status['gripper_angle']], False)
                waypoint_2 = self.calcRelMove([-110, -7, 80, 0, 0, 0], True)
                waypoint_3 = self.calcRelMove([0, 0, 20, 0, 0, 0], True)
                waypoint_4 = self.calcRelMove([0, 0, -50, 0, 0, 0], True)
                waypoint_5 = self.calcRelMove([200, 0, -50, 0, 0, 0], True)

                movel(waypoint_1, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                movel(waypoint_2, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                movel(waypoint_3, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                self.suction_cup_off();  rospy.sleep(1)
                movel(waypoint_4, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                movel(waypoint_5, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                movej(Q_TOP_PLATE)
                self.ARupdateParam(-0.12, 0.0, 0.25, rx=180.0, ry=0.0, rz=180.0)
            

    def specimenAlign(self):
        movel([-405.0, 8.0, 280.0, 90.0, -180.0, 0.0])
        self.movel_z_base(-80)
        self.movel_x_base(15)
        self.movel_x_base(-15)
        self.movel_y_base(-26.49*2)
        self.movel_x_base(15)
        self.movel_x_base(-15)
        self.movel_z_base(80)

    
    '''
        "~/ur_pnp" Topic Protocol (for Doosan-robot control)
        
        Naming rules:
            @ 정의된 숫자 순서에 맞는 위치에 정의 (오름차순)
            @ ACTION_[이름 정의(대문자)] : 0  ~ 10000
                *    0 ~  100: Basic Move
                *  101 ~  200: Doosan-robot I/O Controller
                * 1000 ~ 4000: Relative Move (Translation)
                    - X -> 1000 (0 mm) ~ 1999 (999 mm)
                    - Y -> 2000 (0 mm) ~ 2999 (999 mm)
                    - Z -> 3000 (0 mm) ~ 3999 (999 mm)
            @ TASK_[이름 정의(대문자)]   : 10001  ~ 20000
    '''
    def pnp_cb(self, msg):
        try:
            cmd_dict = json.loads(msg.data)
            self.cmd_protocol = int(cmd_dict['command'])
        except:
            self.cmd_protocol = int(float(msg.data))

        print(self.cmd_protocol)
        self.status['current_work'] = self.cmd_protocol
        self.status_overwrite = "Moving"

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        release_compliance_ctrl()
        self.setVelAcc(50, 50, [150,50], [150,50])
        
        ########################################################################################################################################################
        # ACTION [0]: Home position
        if(self.cmd_protocol   == ACTION_HOME):         
            movej(Q_HOME, 50, 50)
        # ACTION [1]: Back position
        elif(self.cmd_protocol == ACTION_BACK):
            movej(Q_BACK, 50, 50)
        # ACTION [2]: Left position
        elif(self.cmd_protocol == ACTION_LEFT):
            movej(Q_LEFT, 50, 50)
        # ACTION [3]: Right position
        elif(self.cmd_protocol == ACTION_RIGHT):
            movej(Q_RIGHT, 50, 50)
        # ACTION [4]: Approach
        elif(self.cmd_protocol == ACTION_APPROACH):
            movej(Q_TOP_PLATE, 50, 50) # Search pose
            self.ARupdateParam(0.0, -0.12, 0.20)
            self.search_ar_target(4)
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 1st approach
            self.search_ar_target(4)
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 2nd approach
            self.search_ar_target(4)
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 3rd approach
        # ACTION [5]: ALIGN
        elif(self.cmd_protocol == ACTION_ALIGN):
            self.ARupdateParam(0.0, 0.0, 0.2)
            self.search_ar_target(1)
            movel(self.drl_pose, vel=[100,30], acc=[100,30])

        # ACTION [101]: Compressor On
        elif(self.cmd_protocol == ACTION_IO_COMPRESSOR_ON):
            self.compressor_on()
        # ACTION [-101]: Compressor Off
        elif(self.cmd_protocol == ACTION_IO_COMPRESSOR_OFF):
            self.compressor_off()
        # ACTION [102]: Tool-changer Detach
        elif(self.cmd_protocol == ACTION_IO_TOOLCHANGER_DETACH):
            self.toolchanger_detach()
        # ACTION [-102]: Tool-changer Attach
        elif(self.cmd_protocol == ACTION_IO_TOOLCHANGER_ATTACH):
            self.toolchanger_attach()
        # ACTION [105]: Universal Jig X-axis Close
        elif(self.cmd_protocol == ACTION_IO_JIG_X_CLOSE):
            self.jig_x_close()
        # ACTION [-105]: Universal Jig X-axis Open
        elif(self.cmd_protocol == ACTION_IO_JIG_X_OPEN):
            self.jig_x_open()
        # ACTION [106]: Universal Jig Y-axis Close
        elif(self.cmd_protocol == ACTION_IO_JIG_Y_CLOSE):
            self.jig_y_close()
        # ACTION [-106]: Universal Jig Y-axis Open
        elif(self.cmd_protocol == ACTION_IO_JIG_Y_OPEN):
            self.jig_y_open()
        # ACTION [108]: Suction-cup ON
        elif(self.cmd_protocol == ACTION_IO_SUCTIONCUP_ON):
            self.suction_cup_on()
        # ACTION [-108]: Suction-cup OFF
        elif(self.cmd_protocol == ACTION_IO_SUCTIONCUP_OFF):
            self.suction_cup_off()
        # ACTION [201]: Gripper Open (flange output)
        elif(self.cmd_protocol == ACTION_IO_GRIPPER_OPEN):
            self.gripper_open()
        # ACTION [-201]: Gripper Close (flange output)
        elif(self.cmd_protocol == ACTION_IO_GRIPPER_CLOSE):
            self.gripper_close()


        # ACTION [301]: Tool Changer - Get Tool1 from Toolchanger1
        elif(self.cmd_protocol == ACTION_TOOLCHANGE_1_ATTACH):
            self.toolchanger_detach()

            p_tool1_step1 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step1[2] += 300
            p_tool1_step2 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step2[2] +=  20
            p_tool1_step3 = deepcopy(P_TOOLCHANGE_1)
            p_tool1_step4 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step4[1] += -20
            p_tool1_step5 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step5[2] += 300

            self.setVelAcc(100, 100, [400,100], [400,100])
            movel(p_tool1_step1)
            movel(p_tool1_step2)

            self.setVelAcc(50, 50, [50,100], [50,100])
            movel(p_tool1_step3)
            self.movel_z(-10)
            task_compliance_ctrl([100, 100, 1000, 100, 100, 100]);  self.movel_z(50);  rospy.sleep(1);   release_compliance_ctrl()
            self.toolchanger_attach();  rospy.sleep(1)
            movel(p_tool1_step4)
            movel(p_tool1_step5)

            self.status['gripper_type'] = 'Suction'
            

        # ACTION [-301]: Tool Changer - Place Tool1 to the Toolchanger1
        elif(self.cmd_protocol == ACTION_TOOLCHANGE_1_DETACH):

            p_tool1_step1 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step1[1] += -20;    p_tool1_step1[2] += 300
            p_tool1_step2 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step2[1] += -20;    p_tool1_step2[2] += 20
            p_tool1_step3 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step3[1] += -20;    p_tool1_step2[2] += 5
            p_tool1_step4 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step4[0] += 0
            p_tool1_step5 = deepcopy(P_TOOLCHANGE_1)
            p_tool1_step6 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step6[2] += 100
            p_tool1_step7 = deepcopy(P_TOOLCHANGE_1);   p_tool1_step7[2] += 200
            p_tool1_step8 = [-435.8908386230469, -346.9735412597656, 71.24858093261719, 136.8338623046875, 178.95765686035156, 58.61622619628906]
            self.setVelAcc(100, 100, [400,100], [400,100])
            movel(p_tool1_step1)
            movel(p_tool1_step2)

            self.setVelAcc(50, 50, [50,100], [50,100])
            movel(p_tool1_step3)
            task_compliance_ctrl([1000, 4500, 4000, 1000, 1000, 1000])
            movel(p_tool1_step4)
            release_compliance_ctrl()
            movel(p_tool1_step5);       rospy.sleep(1)
            self.toolchanger_detach();  rospy.sleep(1)
            movel(p_tool1_step8)
            
            movel(p_tool1_step6)

            self.setVelAcc(200, 200, [400,100], [400,100])
            movel(p_tool1_step7)

            self.status['gripper_type'] = None
            

        # ACTION [302]: Tool Changer - Get Tool2 from Toolchanger2
        elif(self.cmd_protocol == ACTION_TOOLCHANGE_2_ATTACH):
            self.toolchanger_detach()

            p_tool2_step1 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step1[2] += 300
            p_tool2_step2 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step2[2] +=  20
            p_tool2_step3 = deepcopy(P_TOOLCHANGE_2)
            p_tool2_step4 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step4[1] += -20
            p_tool2_step5 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step5[2] += 300
            self.setVelAcc(100, 100, [400,100], [400,100])
            
            movel(p_tool2_step1)
            movel(p_tool2_step2)
            self.setVelAcc(50, 50, [50,100], [50,100])

            movel(p_tool2_step3)
            self.movel_z(-10)
            task_compliance_ctrl([100, 100, 1000, 100, 100, 100]);  self.movel_z(50); rospy.sleep(1);   release_compliance_ctrl()

            self.toolchanger_attach();  rospy.sleep(1)
            movel(p_tool2_step4)
            movel(p_tool2_step5)

            self.status['gripper_type'] = 'Mechanical'


        # ACTION [-302]: Tool Changer - Place Tool2 to the Toolchanger2
        elif(self.cmd_protocol == ACTION_TOOLCHANGE_2_DETACH):
            
            p_tool2_step1 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step1[1] += -20;    p_tool2_step1[2] += 300
            p_tool2_step2 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step2[1] += -20;    p_tool2_step2[2] += 20
            p_tool2_step3 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step3[1] += -20;    p_tool2_step2[2] += 5
            p_tool2_step4 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step4[0] += 0
            p_tool2_step5 = deepcopy(P_TOOLCHANGE_2)
            p_tool2_step6 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step6[2] += 100
            p_tool2_step7 = deepcopy(P_TOOLCHANGE_2);   p_tool2_step7[2] += 200

            self.setVelAcc(100, 100, [400,100], [400,100])
            movel(p_tool2_step1)
            movel(p_tool2_step2)

            self.setVelAcc(50, 50, [50,100], [50,100])
            movel(p_tool2_step3)
            task_compliance_ctrl([1000, 4500, 4000, 1000, 1000, 1000])

            # movel(p_tool2_step4)
            release_compliance_ctrl()
            movel(p_tool2_step5);       rospy.sleep(1)
            self.toolchanger_detach();  rospy.sleep(1)
            movel(p_tool2_step6)

            self.setVelAcc(100, 100, [400,100], [400,100])
            movel(p_tool2_step7)
            self.setVelAcc(50, 50, [50,100], [50,100])

            self.status['gripper_type'] = None


        # ACTION [1000 ~ 1999]: Trans X (relative move)
        elif(abs(self.cmd_protocol) >= ACTION_TRANS_X and abs(self.cmd_protocol) < ACTION_TRANS_Y):
            sign = self.cmd_protocol / abs(self.cmd_protocol)
            self.movel_x(sign * (abs(self.cmd_protocol) - ACTION_TRANS_X))
        # ACTION [2000 ~ 2999]: Trans Y (relative move)
        elif(abs(self.cmd_protocol) >= ACTION_TRANS_Y and abs(self.cmd_protocol) < ACTION_TRANS_Z):
            sign = self.cmd_protocol / abs(self.cmd_protocol)
            self.movel_y(sign * (abs(self.cmd_protocol) - ACTION_TRANS_Y))
        # ACTION [3000 ~ 4000]: Trans Z (relative move)
        elif(abs(self.cmd_protocol) >= ACTION_TRANS_Z and abs(self.cmd_protocol) < ACTION_TRANS):
            sign = self.cmd_protocol / abs(self.cmd_protocol)
            self.movel_z(sign * (abs(self.cmd_protocol) - ACTION_TRANS_Z))
        

        # Task [10001]: Pick a tensile test specimen
        elif(self.cmd_protocol == TASK_SPECIMEN_PICK):
            self.gripper_open()
            movej(Q_TOP_PLATE, 50, 50)

            self.setVelAcc(50, 50, [100,50], [100,150])
            approachj = [33.12971115112305, -36.94938278198242, -131.39822387695312, -57.41393280029297, -96.33760070800781, 170.20169067382812]
            movej(approachj)

            first_sp= [-425.50738525390625, -15.4998779296875, 100.5030517578125, 90, 90, 180]
            movel(first_sp)

            self.gripper_close()
            
            liftup= [-425.50738525390625, -15.4998779296875, 110.5030517578125, 90, 90, 180]
            movel(liftup)

            backup= [-425.50738525390625, -115.4998779296875, 110.5030517578125, 90, 90, 180]
            movel(backup)
            
            movej(Q_TOP_PLATE, 50, 50)


        # Task [10002]: Search AR_Marker attached to the upper gripper of Instron
        elif(self.cmd_protocol == TASK_INSTRON_SEARCH):
            self.gripper_close()
            movej(Q_TOP_PLATE, 50, 50) 

            self.setVelAcc(30, 30, [100,50], [100,50])
            see_point1j = [81.08692169189453, -0.4761710464954376, -143.7606658935547, -9.412845611572266, 57.22504806518555, -80.97422790527344+360]
            movej(see_point1j)
            
            ar_tag = 0

            self.ARupdateParam(-0.12, 0.0, 0.25, rx=180.0, ry=0.0, rz=180.0); rospy.sleep(2)
            if self.ARsearchFromEEF(ar_tag) == True: 
                self.ARsetReference(ar_tag, 4)
                self.movel_xyz(-178, -90, -180)
                movej([self.status['posj'][0], self.status['posj'][1], self.status['posj'][2], self.status['posj'][3], self.status['posj'][4], self.status['posj'][5] - 180])
                self.movel_xyz(0, 0, 298)
                # self.ARsetReference(ar_tag, 1); rospy.sleep(0.5)
                # self.ARsetReference(ar_tag, 1); rospy.sleep(0.5)
                # self.ARsetReference(ar_tag, 1)
                # self.movel_xyz(-157, -90, 0)
                # self.movel_xyz(0, 0, 220)

	    
        # Task [10003]: Place specimen and go to the monitoring position
        elif(self.cmd_protocol == TASK_INSTRON_MOVEOUT):
            # rospy.sleep(10)
            # movel([0,0,-200,0,0,90], mod = 1, ref = 1)
            self.gripper_open();  rospy.sleep(1.0)
            self.movel_z(-100)
            self.movel_xyz(100, -200, -100)
            # movel([0,0,-200,0,0,90], mod = 1, ref = 1)
            # viewpoint = deepcopy(self.status['posx']);    viewpoint[4] -= 20
            viewpoint = [self.status['posx'][0],self.status['posx'][1],self.status['posx'][2],self.status['posx'][3],self.status['posx'][4]-20,self.status['posx'][5]]
            movel(viewpoint)



        # Task [10004]: SEARCH AND APPROACH TO ''MULTIPLE'' SPECIMENS AND DETACH TO THE BED
        elif(self.cmd_protocol == TASK_DETACH_SPECIMEN):
            self.setVelAcc(30, 30, [30,30], [30,30])
            movej(Q_MULSPECIMEN_SEARCH)
            self.gripper_open()

            object_count=1
            
            ## Publish Flag to 'snu_2d_vision.py' node
            self.vision_pub.publish(30002)
            listener = tf.TransformListener()
            while True:
                try:
                    target_frame_name = 'specimen_table_' + str(object_count)
                    reference_frame_name = 'base_0'
                    print "Searching specimen ..."
                    print("Target frame: "    + target_frame_name)
                    print("Reference frame: " + reference_frame_name)
                    print "Trying to search the specimen: %s ..."%target_frame_name

                    listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(8.0))
                    (trans,rot) = listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time())
                    print(trans, rot)
                    self.update_target_pose(trans, rot)
                    self.updateEulZYZ()
                    self.drl_pose = deepcopy(posx(self.target_pose.position.x, self.target_pose.position.y, 332 , -181.3, -180, -self.eulerZYZ[2]-self.eulerZYZ[0]))
                    print('Target DRL Pose: ' , self.drl_pose)

                    print('search complete')
                    movel(self.drl_pose)
                    specimen_angle = self.drl_pose[5]
                    #SHAKING
                    self.movel_y(-60)
                    self.movel_z(102, [100, 100], [100, 100]) #go down 94 for debug T4 -> 103mm
                    while True:
                        self.specimen_shaking()
                        if self.status['force'][1] > 30:
                            self.gripper_open()
                            self.movel_y(-60)
                            continue
                        else :
                            self.gripper_open()
                            release_compliance_ctrl()
                            break
                    rospy.sleep(1)
                    self.movel_z(2)
                    self.gripper_close()
                    self.movel_z(-102,[100, 100], [100, 100]) #go up
                    movej(Q_MULSPECIMEN_SEARCH)

                    if object_count == 1:
                        break

                    object_count= object_count+1

                except (Exception):
                    print "[ERROR]: The Target(TF) is not Detected !!!"
                    print("Specimen count :{}".format(object_count-1))
                    break

        # Task [10005]: SEARCH ONE SPECIMEN AND PICK UP
        elif(self.cmd_protocol == TASK_SEARCH_PICK_SPECIMEN):
            self.gripper_open()
            self.setVelAcc(30, 30, [50,50], [50,50])
            movej(Q_MULSPECIMEN_SEARCH)

            object_count=1
            
            ## Publish Flag to 'snu_2d_vision.py' node
            self.vision_pub.publish(30002)
            listener = tf.TransformListener()

            target_frame_name = 'specimen_table_' + str(object_count)
            reference_frame_name = 'base_0'
            print "Searching specimen ..."
            print("Target frame: "    + target_frame_name)
            print("Reference frame: " + reference_frame_name)
            print "Trying to search the specimen: %s ..."%target_frame_name
            listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(5.0))
            (trans,rot) = listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
            self.update_target_pose(trans, rot)
            self.updateEulZYZ()
            self.drl_pose = deepcopy(posx(self.target_pose.position.x, self.target_pose.position.y, 332 , -181.3, -180, -self.eulerZYZ[2]-self.eulerZYZ[0]))
            print('Target DRL Pose: ' , self.drl_pose)
            print('search complete')

            movel(self.drl_pose)
            self.movel_z(104, [100, 100], [100, 100]) #go down 95 for development 
            self.gripper_close()
            self.movel_z(-104,[100, 100], [100, 100]) #go up
            movej(Q_MULSPECIMEN_SEARCH)
                

        # Task [10006]: AFTER ATTACHING SENSOR PICK SPECIMEN AND PLACE ON RACK
        elif(self.cmd_protocol == TASK_SPECIMEN_TO_RACK):
            '''
            self.gripper_open()
            self.setVelAcc(30, 30, [30, 30], [30, 30])
            Q_SPECIMEN_RETRACT = [35.044342041015625, 9.633670806884766, -137.1417694091797, -0.0, -52.49190902709961, 35.044342041015625]
            movej(Q_SPECIMEN_RETRACT)
            movel(P_AFTER_ATTACH)
            self.movel_z_base(-23)
            # rospy.sleep(5)
            self.gripper_close()
            self.movel_y_base(10)
            # movej(Q_MULSPECIMEN_SEARCH)
            self.movel_z_base(50)
            '''

            # movel(P_PLACE_INITIAL)
            movej(Q_PLACE_INITIAL)
            movel(P_PLACE_INCLINE)
            movel(P_PLACE_RACK_1)
            self.move_lack_place()

            # movej(Q_MULSPECIMEN_SEARCH)

            # self.specimenAlign()

            # movel(P_PICK_INITIAL)
            # movel(P_PICK_INCLINE)
            # movel(P_PICK_RACK_2)
            # self.move_lack_pick()

            movej(Q_MULSPECIMEN_SEARCH)


        # Task [10007]: Specimen pick and place at rack TEST
        elif(self.cmd_protocol == TASK_SPECIMEN_FROM_RACK):
            self.gripper_open()
            self.setVelAcc(70, 70, [50,50], [50,50])
            movej(Q_MULSPECIMEN_SEARCH)
            movel(P_PICK_INITIAL)
            movel(P_PICK_INCLINE)
            movel(P_PICK_RACK_1)
            self.move_lack_pick()
            movej(Q_MULSPECIMEN_SEARCH)


        # Task [10008]: Alignment task
        elif(self.cmd_protocol == TASK_RACK_ALIGN):
            self.specimenAlign()            


        # Task [10010]: "TASK_3DP_BED_IN"   - (3DP-#1~4 Bed) Jig -> Printer 
        elif(self.cmd_protocol == TASK_3DP_BED_IN):
            tag_id_bed   = self.searchARTagFromRobot()
            tag_id_stage = self.searchARTagFromRight()
            print("[DEBUG] Bed: {}, Stage: {}".format(tag_id_bed, tag_id_stage))
            if tag_id_stage != None and tag_id_bed != None:
                self.moveBedFromRobotToStage(tag_id_bed=tag_id_bed, tag_id_stage=tag_id_stage)

        # Task [-10010]: "TASK_3DP_BED_OUT" - (3DP-#1~4 Bed) Printer -> Jig 
        elif(self.cmd_protocol == TASK_3DP_BED_OUT):
            tag_id_bed = self.searchARTagFromRight()
            print("[DEBUG] Bed: {}".format(tag_id_bed))
            if tag_id_bed != None:
                self.moveBedFromStageToRobot(tag_id_bed=tag_id_bed, tag_id_stage=None)

        # Task [10011]: "TASK_3DP_1_BED_IN"   - (3DP-#1 Bed) Jig -> Printer 
        elif(self.cmd_protocol == TASK_3DP_1_BED_IN):
            tag_id_stage = 1
            tag_id_bed = tag_id_stage + 4
            self.moveBedFromRobotToStage(tag_id_bed=tag_id_bed, tag_id_stage=tag_id_stage)
        # Task [-10011]: "TASK_3DP_1_BED_OUT" - (3DP-#1 Bed) Printer -> Jig 
        elif(self.cmd_protocol == TASK_3DP_1_BED_OUT):
            tag_id_bed = 1 + 4
            self.moveBedFromStageToRobot(tag_id_bed=tag_id_bed, tag_id_stage=None)

        # Task [10012]: "TASK_3DP_2_BED_IN"   - (3DP-#2 Bed) Jig -> Printer 
        elif(self.cmd_protocol == TASK_3DP_2_BED_IN):
            tag_id_stage = 2
            tag_id_bed = tag_id_stage + 4
            self.moveBedFromRobotToStage(tag_id_bed=tag_id_bed, tag_id_stage=tag_id_stage)
        # Task [-10012]: "TASK_3DP_2_BED_OUT" - (3DP-#2 Bed) Printer -> Jig 
        elif(self.cmd_protocol == TASK_3DP_2_BED_OUT):
            tag_id_bed = 2 + 4
            self.moveBedFromStageToRobot(tag_id_bed=tag_id_bed, tag_id_stage=None)

        # Task [10013]: "TASK_3DP_3_BED_IN"   - (3DP-#3 Bed) Jig -> Printer 
        elif(self.cmd_protocol == TASK_3DP_3_BED_IN):
            tag_id_stage = 3
            tag_id_bed = tag_id_stage + 4
            self.moveBedFromRobotToStage(tag_id_bed=tag_id_bed, tag_id_stage=tag_id_stage)
        # Task [-10013]: "TASK_3DP_3_BED_OUT" - (3DP-#3 Bed) Printer -> Jig 
        elif(self.cmd_protocol == TASK_3DP_3_BED_OUT):
            tag_id_bed = 3 + 4
            self.moveBedFromStageToRobot(tag_id_bed=tag_id_bed, tag_id_stage=None)

        # Task [10014]: "TASK_3DP_4_BED_IN"   - (3DP-#4 Bed) Jig -> Printer 
        elif(self.cmd_protocol == TASK_3DP_4_BED_IN):
            tag_id_stage = 4
            tag_id_bed = tag_id_stage + 4
            self.moveBedFromRobotToStage(tag_id_bed=tag_id_bed, tag_id_stage=tag_id_stage)
        # Task [-10014]: "TASK_3DP_4_BED_OUT" - (3DP-#4 Bed) Printer -> Jig 
        elif(self.cmd_protocol == TASK_3DP_4_BED_OUT):
            tag_id_bed = 4 + 4
            self.moveBedFromStageToRobot(tag_id_bed=tag_id_bed, tag_id_stage=None)


        # Task [10020]: "DEMO_COLOR_SENSOR_HANDLE" - Color sensor handling demo
        elif(self.cmd_protocol == TASK_ATTACH_SENSOR):
            movej(Q_HOME)
            self.getColorSensor(1); movej(Q_COLOR_SENSOR_TRAY_RETRACT); movel(P_COLOR_SENSOR_TRAY_LEFT);  movel(P_COLOR_SENSOR_1); self.movel_z(5); self.suction_cup_off(); self.movel_z(-5)
            self.getColorSensor(2); movej(Q_COLOR_SENSOR_TRAY_RETRACT); movel(P_COLOR_SENSOR_TRAY_LEFT);  movel(P_COLOR_SENSOR_2); self.movel_z(5); self.suction_cup_off(); self.movel_z(-5)
            self.getColorSensor(3); movej(Q_COLOR_SENSOR_TRAY_RETRACT); movel(P_COLOR_SENSOR_TRAY_LEFT);  movel(P_COLOR_SENSOR_3); self.movel_z(5); self.suction_cup_off(); self.movel_z(-5)
            self.getColorSensor(4); movej(Q_COLOR_SENSOR_TRAY_RETRACT); movel(P_COLOR_SENSOR_TRAY_LEFT);  movel(P_COLOR_SENSOR_4); self.movel_z(5); self.suction_cup_off(); self.movel_z(-5)
            self.getColorSensor(5); movej(Q_COLOR_SENSOR_TRAY_RETRACT); movel(P_COLOR_SENSOR_TRAY_RIGHT); movel(P_COLOR_SENSOR_5); self.movel_z(5); self.suction_cup_off(); self.movel_z(-5)
            self.getColorSensor(6); movej(Q_COLOR_SENSOR_TRAY_RETRACT); movel(P_COLOR_SENSOR_TRAY_RIGHT); movel(P_COLOR_SENSOR_6); self.movel_z(5); self.suction_cup_off(); self.movel_z(-5)
            self.getColorSensor(7); movej(Q_COLOR_SENSOR_TRAY_RETRACT); movel(P_COLOR_SENSOR_TRAY_RIGHT); movel(P_COLOR_SENSOR_7); self.movel_z(5); self.suction_cup_off(); self.movel_z(-5)
            self.getColorSensor(8); movej(Q_COLOR_SENSOR_TRAY_RETRACT); movel(P_COLOR_SENSOR_TRAY_RIGHT); movel(P_COLOR_SENSOR_8); self.movel_z(5); self.suction_cup_off(); self.movel_z(-5)
            movej(Q_HOME)
        

        # ACTION [10021 ~ 10030]: Attach sensor (1~8) to the specimen
        elif(abs(self.cmd_protocol) >= TASK_ATTACH_SENSOR and abs(self.cmd_protocol) < TASK_ATTACH_SENSOR+10):
            Q_SPECIMEN_RETRACT = [35.044342041015625, 9.633670806884766, -137.1417694091797, -0.0, -52.49190902709961, 35.044342041015625]
            P_SPECIMEN_SENSOR_ATTACH = [-262.343505859375, -179.81179809570312, 187.67666625976562, 180, 180, 0]

            waypoint_1 = deepcopy(P_SPECIMEN_SENSOR_ATTACH);  waypoint_1[2] += 30
            waypoint_2 = deepcopy(P_SPECIMEN_SENSOR_ATTACH)

            sensor_number = abs(self.cmd_protocol) - TASK_ATTACH_SENSOR
            print(sensor_number)
            movej(Q_SPECIMEN_RETRACT)
            self.getColorSensor(sensor_number)
            movej(Q_SPECIMEN_RETRACT)
            movel(waypoint_1)
            movel(waypoint_2)
            self.suction_cup_off();  rospy.sleep(1)
            self.movel_z(-10)
            movej(Q_SPECIMEN_RETRACT)


        # Task [10031]: Adhesive saver in
        elif(self.cmd_protocol == TASK_ADHESIVE_SAVER_IN):
            init_posj = [9.329447746276855, -3.420710563659668, -127.80497741699219, -0.0, -48.774375915527344, -170.6704864501953]
            movej(init_posj);   self.gripper_open()
            self.movel_z(100);  self.gripper_close()
            self.movel_xyz(-5, -5, -5);   self.movel_z(-100)

            waypoint_0 = [-363.7647399902344, -177.87579345703125, 186.26629638671875, 180, -180, 90]
            waypoint_1 = deepcopy(waypoint_0);  waypoint_1[0]+=30;  waypoint_1[2]+=30
            waypoint_2 = deepcopy(waypoint_1);  waypoint_2[2]-=30
            waypoint_3 = deepcopy(waypoint_0)
            waypoint_4 = deepcopy(waypoint_3);  waypoint_4[0]-=10
            movel(waypoint_1)
            movel(waypoint_2)
            movel(waypoint_3)
            # task_compliance_ctrl([10, 10, 10, 2000, 2000, 2000])
            # movel(waypoint_4)
            # release_compliance_ctrl()
            self.gripper_open()
            movel(waypoint_1)


        # Task [-10031]: Adhesive saver out
        elif(self.cmd_protocol == TASK_ADHESIVE_SAVER_OUT):
            init_posj = [33.17411804199219, 4.681657791137695, -134.74765014648438, 0.07327302545309067, -50.032894134521484, -56.88725280761719]
            movej(init_posj);   self.gripper_open()
            goal_posj = [9.329447746276855, -3.420710563659668, -127.80497741699219, -0.0, -48.774375915527344, -170.6704864501953]

            waypoint_0 = [-363.7647399902344, -177.87579345703125, 188.0, 180, -180, 90]
            waypoint_1 = deepcopy(waypoint_0);  waypoint_1[0]+=30;  waypoint_1[2]+=30
            waypoint_2 = deepcopy(waypoint_0)
            waypoint_3 = deepcopy(waypoint_2);  waypoint_3[0]-=3
            waypoint_4 = deepcopy(waypoint_3);  waypoint_4[0]+=50
            waypoint_5 = deepcopy(waypoint_4);  waypoint_5[2]+=50
            movel(waypoint_1)
            movel(waypoint_2)
            movel(waypoint_3)
            self.gripper_close()
            movel(waypoint_4)
            movel(waypoint_5)
            movej(goal_posj)
            self.movel_xyz(-5, -5, 100)
            # task_compliance_ctrl([10, 10, 10, 2000, 2000, 2000])
            self.movel_xyz(5, 7, 0)
            # release_compliance_ctrl()
            self.gripper_open()
            self.movel_z(-100)


        # Task [10032]: Drop adhesive
        elif(self.cmd_protocol == TASK_ADHESIVE_DROP):
            # self.setVelAcc(50, 50, [50,100], [50,100])
            # init_posx = [-428.15545654296875, -181.14376831054688, 261.2680969238281, 180, 180, 90]
            # movel(init_posx)
            release_compliance_ctrl()
            self.setVelAcc(50, 50, [50,100], [50,100])
            init_posj = [27.18817138671875, 1.201263189315796, -122.7035140991211, -0.0, -58.497928619384766, -62.811859130859375]

            movej(init_posj,50,50)
            self.movel_z(97)

            k = 100
            g = 9.81
            
            eef_weight = self.status['force'][2]
            print(self.status['force'])
            print(task_compliance_ctrl([3000, 3000, 2000, 2000, 2000, 2000]))
            print 'comliance working'

            while(True):
                print(self.status['force'][2])
                if self.status['force'][2] > 7:  #set force in N
                    contact_posx = posx(self.status['posx'][0], self.status['posx'][1], self.status['posx'][2]+5, 180, 180, 0)
                    release_compliance_ctrl()
                    print("Z position: {}".format(contact_posx[2]))
                    break
                self.movel_z_base(-0.2)
            movej(init_posj,50,50)


        # Task [10033]: Move specimen to the center of the working table
        elif(self.cmd_protocol == TASK_SEPCIMEN_TO_CENTER):
            self.setVelAcc(50, 50, [150,50], [150,50])
            
            Q_SPECIMEN_RETRACT = [35.044342041015625, 9.633670806884766, -137.1417694091797, -0.0, -52.49190902709961, 35.044342041015625]
            P_SPECIMEN_GLUE_SUCTION = [-357.6464538574219, -122.03623962402344, 186.5482940673828, 180, -180, 180]
            P_SPECIMEN_GLUE_MECH    = [-324.44671630859375, -179.0755157470703, 236.5482940673828, 0, 180, 90]
            P_SPECIMEN_START = [-280.0, -174, 187, 24.260906219482422, -178.50173950195312, 25.866914749145508]

            waypoint_1 = deepcopy(P_SPECIMEN_GLUE_MECH);   waypoint_1[0]+=30
            waypoint_2 = deepcopy(waypoint_1);             waypoint_2[2]-=50
            waypoint_3 = deepcopy(waypoint_2);             waypoint_3[0]-=80
            waypoint_4 = deepcopy(P_SPECIMEN_START);       waypoint_4[1]+=30;  waypoint_4[2]+=20
            waypoint_5 = deepcopy(waypoint_4);             waypoint_5[1]-=30;  waypoint_5[2]-=20

            movej(Q_SPECIMEN_RETRACT)
            self.movel_z_base(-30)
            movel(waypoint_4)
            movel(waypoint_5)
            task_compliance_ctrl([3000,200,3000,3000,3000,3000])
            self.movel_y_base(-20)
            release_compliance_ctrl()
            self.gripper_open();    rospy.sleep(1)

            self.movel_z(-50)
            movej(Q_SPECIMEN_RETRACT)

            
        # Task [10034]: Move specimen to the left-hand-side of the working table
        elif(self.cmd_protocol == TASK_SPECIMEN_TO_LEFT):
            self.setVelAcc(50, 50, [150,50], [150,50])
            Q_SPECIMEN_RETRACT = [35.044342041015625, 9.633670806884766, -137.1417694091797, -0.0, -52.49190902709961, 35.044342041015625]
            P_SPECIMEN_GLUE_SUCTION = [-357.6464538574219, -122.03623962402344, 186.5482940673828, 180, -180, 180]
            P_SPECIMEN_GLUE_MECH    = [-324.44671630859375, -179.0755157470703, 236.5482940673828, 0, 180, 90]
            P_SPECIMEN_START = [-330.6361999511719, -178.67942810058594, 185.79730224609375, 24.260906219482422, -178.50173950195312, 25.866914749145508]
            
            # waypoint_1 = deepcopy(P_SPECIMEN_GLUE_MECH);   waypoint_1[0]+=30
            waypoint_1 = deepcopy(P_SPECIMEN_GLUE_MECH);   waypoint_1[0]+=80
            waypoint_2 = deepcopy(waypoint_1);             waypoint_2[2]-=50
            waypoint_3 = deepcopy(waypoint_2);             waypoint_3[0]-=130
            waypoint_4 = deepcopy(P_SPECIMEN_START);       waypoint_4[1]+=30;  waypoint_4[2]+=50
            waypoint_5 = deepcopy(waypoint_4);             waypoint_5[1]-=30;  waypoint_5[2]-=50

            movej(Q_SPECIMEN_RETRACT)
            movel(waypoint_1)
            movel(waypoint_2)
            movel(waypoint_3)
            movel(waypoint_2)
            movel(waypoint_1)
            movej(Q_SPECIMEN_RETRACT)


        # Task [10035]: Move specimen to the right-hand-side of the working table
        elif(self.cmd_protocol == TASK_SPECIMEN_TO_RIGHT):
            self.setVelAcc(50, 50, [150,50], [150,50])
            Q_SPECIMEN_RETRACT = [35.044342041015625, 9.633670806884766, -137.1417694091797, -0.0, -52.49190902709961, 35.044342041015625]
            P_SPECIMEN_GLUE_SUCTION = [-357.6464538574219, -232.03623962402344, 186.0482940673828, 180, -180, 180]
            P_SPECIMEN_GLUE_MECH    = [-374.74671630859375, -179.0755157470703, 186.5482940673828, 0, 180, 90]
            P_SPECIMEN_SENSOR_SUCTION = [-270.86053466796875, -232.03623962402344, 185.5482940673828, 90, 180, -90]
            P_SPECIMEN_SENSOR_ATTACH = [-263.843505859375, -179.81179809570312, 187.67666625976562, 180, 180, 0]

            movej(Q_SPECIMEN_RETRACT)
            waypoint_1 = deepcopy(P_SPECIMEN_GLUE_SUCTION);  waypoint_1[2] += 50
            waypoint_2 = deepcopy(P_SPECIMEN_GLUE_SUCTION)
            waypoint_3 = deepcopy(waypoint_2);               waypoint_3[0] += 100
            waypoint_4 = deepcopy(waypoint_3);               waypoint_4[2] += 50
            waypoint_5 = deepcopy(P_SPECIMEN_GLUE_MECH);     waypoint_5[2] += 50
            waypoint_6 = deepcopy(P_SPECIMEN_GLUE_MECH);     waypoint_6[0] += 10
            waypoint_7 = deepcopy(waypoint_6);               waypoint_7[0] += 110
            waypoint_8 = deepcopy(waypoint_7);               waypoint_8[2] += 50
            ## Sensor 부착
            waypoint_9 = deepcopy(P_SPECIMEN_SENSOR_ATTACH); waypoint_9[2] += 30
            waypoint_10 = deepcopy(P_SPECIMEN_SENSOR_ATTACH)
            # waypoint_11 = deepcopy(P_SPECIMEN_SENSOR_SUCTION)
            # waypoint_12 = deepcopy(waypoint_1);              waypoint_12[0] -= 15

            movej(Q_SPECIMEN_RETRACT)
            movel(waypoint_1)
            movel(waypoint_2)
            self.suction_cup_on();  rospy.sleep(1)
            movel(waypoint_3)
            self.suction_cup_off(); rospy.sleep(1)
            movel(waypoint_4)
            movel(waypoint_5)   
            movel(waypoint_6)
            movel(waypoint_7)
            movel(waypoint_8)
            movej(Q_SPECIMEN_RETRACT)


        # Task [10036]: Specimen is ready
        elif(self.cmd_protocol == TASK_SPECIMEN_READY):
            rospy.sleep(30)
            Q_SPECIMEN_RETRACT = [35.044342041015625, 9.633670806884766, -137.1417694091797, -0.0, -52.49190902709961, 35.044342041015625]
            P_SPECIMEN_SENSOR_ATTACH = [-263.843505859375, -179.81179809570312, 187.67666625976562, 180, 180, 0]

            waypoint_1 = deepcopy(P_SPECIMEN_SENSOR_ATTACH); waypoint_1[2] += 30

            movej(Q_SPECIMEN_RETRACT)
            movel(waypoint_1)
            self.movel_xyz(-4, 51, 31)
            self.suction_cup_on();  rospy.sleep(1)
            self.movel_x(-30)
            self.suction_cup_off()
            movej(Q_SPECIMEN_RETRACT)

        # Task [20001]: Testing compliance mode using scale -> (F = -kx // k=10;10;100 , x=10;10;100)
        elif(self.cmd_protocol == TASK_TEST_COMPLIANCE):
            self.setVelAcc(50, 50, [50,100], [50,100])

            init_posj = Q_BACK
            movej(init_posj)

            self.movel_z(280)
            k = 10
            x = 10
            g = 9.81
            
            eef_weight = self.status['force'][2]
            task_compliance_ctrl([100, 100, 10, 1000, 1000, 1000])

            ##contact point 정의 필요 movel할 것
            while(True):
                if self.status['force'][2] == 0.0:  #set force in N
                    contact_posx = posx(self.status['posx'][0], self.status['posx'][1], self.status['posx'][2]+5, 180, 180, 0)
                    release_compliance_ctrl()
                    print("Z position: {}".format(contact_posx[2]))
                    break

            for k in range(10,1000,10):
                for x in range(10,100,10):
                    task_compliance_ctrl([100, 100, k, 1000, 1000, 1000])
                    self.movel_xyz(0, 0, x, velx=[10,10])
                    print("----------------------------------")
                    print("k: {}".format(k), "x: {}".format(x))
                    # print("Expected mass -> {} [g]".format(k*x/g))
                    # print("End-effector mass -> {} [g]".format(-eef_weight/g * 1000))
                    print("Result mass -> {} g".format((self.toolforce_max - eef_weight)/g * 1000))

                    release_compliance_ctrl()
                    movel(contact_posx, vel=[20,20], acc=[100,50])
                    self.toolforce_max = 0.0

        # TEST: Color sensor handling [20004]
        elif(self.cmd_protocol == 20004):
            # movel([-292.6132507324219, 113.29225158691406, 186.91656494140625, 180, -180, 90])
            self.getColorSensor(1)
            # movej(Q_COLOR_SENSOR_TRAY_RETRACT)
            # movel(P_COLOR_SENSOR_TRAY)
            # movel(P_COLOR_SENSOR_1)
            # movel(P_COLOR_SENSOR_2)
            # movel(P_COLOR_SENSOR_3)
            # movel(P_COLOR_SENSOR_4)
            # movel(P_COLOR_SENSOR_5)
            # movel(P_COLOR_SENSOR_6)
            # movel(P_COLOR_SENSOR_7)
            # movel(P_COLOR_SENSOR_8)

            # self.movel_y(-4*dx)
            # for i in range(n-1):
            #     self.movel_z(5)
            #     self.suction_cup_on()
            #     self.movel_z(-5)
            #     self.movel_z(5)
            #     self.suction_cup_off()
            #     self.movel_z(-5)
            #     self.movel_y(dx)
                
            # movel(P_COLOR_SENSOR_TRAY)
            # movej(Q_COLOR_SENSOR_TRAY_RETRACT)


        # TEST: AR tag [20005]
        elif(self.cmd_protocol == 20005):
            self.setVelAcc(100, 100, [100,50], [100,50])

            movej(Q_HOME)
            # movej(Q_SEARCH_3DP_RIGHT)
            ar_tag_number = 6
            self.ARupdateParam(-0.12, 0.0, 0.30, rx=180.0, ry=0.0, rz=180.0); rospy.sleep(1)
            self.ARsetReference(ar_tag_number, 5)


        # ACTION [20010]: test
        elif(self.cmd_protocol == 20010):
            movej(Q_HOME)
            task_compliance_ctrl([5000, 5000, 100, 5000, 5000, 5000])
            print "1"
            self.movel_z(20)
            print "2"
            release_compliance_ctrl()
            print "3"


        ## 기계시스템설계2 최영진 학생 코드   `
        elif(self.cmd_protocol == MSD_YJ_1):
            ## Action 1. attaching fabric to the end-effector
            movej(Q_TOP_PLATE, 50, 50)  # Search pose
            ar_tag_fabric = 0
            self.ARupdateParam(-0.12, 0.0, 0.20, rx=180.0, ry=0.0, rz=180.0); rospy.sleep(1)
            self.ARsetReference(ar_tag_fabric, 4)
            if self.ARsearchFromEEF(ar_tag_fabric) == True:
                self.setVelAcc(50, 50, [50, 100], [50, 100])
                # task_compliance_ctrl([4500, 4500, 1000, 1000, 1000, 1000])
                self.movel_x(-120, ref=DR_BASE)
                task_compliance_ctrl([100, 100, 1000, 100, 100, 100])
                self.movel_z(-50, ref=DR_BASE)
                self.movel_z(+50, ref=DR_BASE)
                release_compliance_ctrl()
                # release_compliance_ctrl()

                self.movel_x(120, ref = DR_BASE)
                task_compliance_ctrl([100, 100, 1000, 100, 100, 100])
                self.rotate_on()
                self.movel_x(120, ref = DR_BASE)
                self.movel_y(-100, ref = DR_BASE)
                self.movel_x(-120, ref = DR_BASE)
                self.movel_y(-100, ref = DR_BASE)
                release_compliance_ctrl()
                self.rotate_off()
                
                ## Action 2. move to the target spot
                ## self.setVelAcc(100, 100, [400, 100], [400, 100])
                ## movej(Q_TOP_PLATE, 50, 50)

                ## ar_tag_target = 6
                ## self.ARupdateParam(-0.12, 0.0, 0.20, rx=180.0, ry=0.0, rz=180.0); rospy.sleep(1)
                ## self.ARsetReference(ar_tag_target, 4)


        ##CONTROL IDIM_GRIPPER (30000)
        elif(self.cmd_protocol == IDIM_CONTROL_TEST):
            ##publish_once
            while self.gripper_pub.get_num_connections()<1:
                self.gripper_pub.publish(True)
                print("interface to gripper success")


        ##AR TAG FLIP TEST
        elif(self.cmd_protocol == 50000):
            movej([0.0, 0.0, -90.0, 0.0, -90.0, 180.0])
            



        ########################################################################################################################################################
        set_robot_mode(ROBOT_MODE_MANUAL)
        release_compliance_ctrl()

        self.status_overwrite = "Standby"
        self.status['recent_work'] = self.status['current_work']
        self.status['current_work'] = None
    
        

if __name__=='__main__':
    cobot = DeviceClass_Cobot(device_name="cobot")
    sleep(3.0)
    while not rospy.is_shutdown():
        cobot.publishStatus()
        sleep(0.5)
        pass
    #     # if(idim.dsr_status == 2):
    #     #     idim.robot_status = "running"
    #     #elif(idim.dsr_status == 1 and idim.robot_status == "running"):
    #     #    idim.robot_status = "done"
    #     #else:
    #     #    idim.robot_status = "waiting"
    #     # idim.status_pub.publish(URStatus(status=idim.robot_status, arm_status = idim.joints_state))
    #     rospy.sleep(0.1)

    # set_robot_mode(ROBOT_MODE_MANUAL)
    
