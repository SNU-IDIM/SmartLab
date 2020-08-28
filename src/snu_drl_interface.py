#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_IDIM_ASMR/common/imp"%HOME_DIR)) )
from IDIM_header import *
from IDIM_framework import *
from pytesseract import *

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False
  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  return True


class DRLInterface():
    def __init__(self, ros_node_name="snu_drl_commander"):
        self.dsr_flag = None
        self.joints_state = None
        self.robot_status = "waiting"
        self.target_pose = Pose()
        self.drl_pose = Q_TOP_PLATE
        self.eulerZYZ = np.zeros(3)
        self.cmd_protocol = ACTION_HOME

        self.toolforce          = []
        self.toolforce_max      = 0.0
        self.toolforce_max_flag = False
        
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
        
        rospy.init_node(ros_node_name, anonymous=True)
        self.listener = tf.TransformListener()
        rospy.Subscriber("ur_pnp", String, self.pnp_cb, queue_size=1)
        rospy.Subscriber("dsr/state", RobotState, self.dsr_state_cb, queue_size=1)
        rospy.Subscriber("dsr/joint_states", JointState, self.current_status_cb, queue_size=1)
        self.image_sub = rospy.Subscriber("/R_001/camera/color/image_raw",Image,self.vision_cb)
        self.pnp_pub    = rospy.Publisher("ur_pnp", String, queue_size=1)
        self.status_pub = rospy.Publisher("ur_status", URStatus, queue_size=1)
        self.vision_pub = rospy.Publisher("vision_2d_flag",Int32, queue_size=1)

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        rospy.sleep(1)
        self.robot_status = "working"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))
        # movej(Q_TOP_PLATE, 50, 50)
        self.robot_status = "done"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))

        
    '''
        dsr_state_cb: "~/dsr/state" topic callback function (update dsr_flag)
    '''
    def dsr_state_cb(self, msg):
        self.dsr_flag = msg.robot_state
        self.current_posx = msg.current_posx
        self.toolforce = msg.actual_ett
        # print(self.toolforce[0], self.toolforce[1], self.toolforce[2])
        # print(self.current_posx)

        ## Initialize Maximum tool force
        if not self.toolforce_max_flag:
            self.toolforce_max = self.toolforce[2]
            self.toolforce_max_flag = True
        
        ## Capture Maximum tool force
        if self.toolforce_max < self.toolforce[2]:
            self.toolforce_max = self.toolforce[2]
            # print(self.toolforce_max)
        

    '''
        current_status_cb: update "~/ur_status" from "~/dsr/joint_state"
    '''
    def current_status_cb(self, data):
        self.joints_state = data


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
        update_cmd_pose: update 'target_pose' to feed 'movel' function for Doosan-robot
            @ input 1: geometry_msgs/Vector3 trans
            @ input 2: geometry_msgs/Quaternion rot
    '''
    def update_cmd_pose(self, trans, rot):
        self.target_pose.position.x    = M2MM(trans[0])# + self.offset_x) # 보정 @(arm -> 측면)
        self.target_pose.position.y    = M2MM(trans[1])# + self.offset_y) # 보정 @(arm -> 정면)
        self.target_pose.position.z    = M2MM(trans[2])# + self.offset_z)
        self.target_pose.orientation.x = rot[0]
        self.target_pose.orientation.y = rot[1]
        self.target_pose.orientation.z = rot[2]
        self.target_pose.orientation.w = rot[3]
        print(self.target_pose)


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
        print('The Euler angles are calculated:', self.eulerZYZ)


    '''
        search_ar_target: lookupTransform to get AR_Target (calculated from AR_Marker)
            @ input 1: int ar_tag_number (ex - 0, 1, 2, 3, ...)
    '''
    def search_ar_target(self, ar_tag_number):
        target_frame_name = 'ar_target_' + str(ar_tag_number)
        reference_frame_name = 'base_0'
        print "Searching AR tag ..."
        print("Target frame: "    + target_frame_name)
        print("Reference frame: " + reference_frame_name)
        listener = tf.TransformListener()
        try:
            print "Trying to search the target: %s ..."%target_frame_name
            self.listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(1.0))
            (trans,rot) = self.listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
            self.update_cmd_pose(trans, rot)
            self.updateEulZYZ()
            self.drl_pose = deepcopy(posx(self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z ,self.eulerZYZ[0], self.eulerZYZ[1], self.eulerZYZ[2]))
            print('Target DRL Pose: ' , self.drl_pose)
        except (Exception):
            print "[ERROR]: The Target(TF) is not Detected !!!"
            pass


    '''
        UpdateParam: Updating parameters for target pose w.r.t. AR_Marker
            @ input 1: double dx [m]
            @ input 2: double dy [m]
            @ input 3: double dz [m]
    '''
    def UpdateParam(self, dx, dy, dz, rx=DEFAULT_OFFSET_FROM_TARGET_RX, ry=DEFAULT_OFFSET_FROM_TARGET_RY, rz=DEFAULT_OFFSET_FROM_TARGET_RZ):
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/x', dx)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/y', dy)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/z', dz)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/rx', rx)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/ry', ry)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/rz', rz)
        rospy.sleep(2)
    

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
    def move_xyz(self, x, y, z, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_TOOL, mod=DR_MV_MOD_REL): # distance [mm]
        movel(posx(x, y, z, 0, 0, 0), vel=velx, acc=accx, ref=ref, mod=mod)
    def movel_xyzjoint(self,x,y,z,jz1,jy,jz2,velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_TOOL, mod=DR_MV_MOD_REL): # distance [mm], angle [degree]
        movel(posx(x, y, z, jz1, jy, jz2), vel=velx, acc=accx, ref=ref, mod=mod)
    def rotate_z(self, jz, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL):
        movel(posx(0,0,0,0,0,jz), vel=velx, acc=accx, ref=ref, mod=mod)

    def move_lack_pick(self):
        self.gripper_open()
        movel(posx(0, -20, -20, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)
        self.gripper_close()
        movel(posx(0, -1, 1, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(posx(0, 20, 20, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)
    def move_lack_place(self):
        self.gripper_close()
        movel(posx(0, -20, -20, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)
        self.gripper_open()
        movel(posx(0, -40, 40, 0, 0, 0), vel=DSR_DEFAULT_JOG_VELX, acc=DSR_DEFAULT_JOG_ACCX, ref=DR_BASE, mod=DR_MV_MOD_REL)

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

    def getBedFromPrinterToJig(self, printer_number):
        bed_number = printer_number + 4
        self.setVelAcc(30, 30, [100,50], [100,50])
        self.jig_x_open();  self.jig_y_open();  rospy.sleep(1)
        movej(Q_SEARCH_3DP_RIGHT)

        self.UpdateParam(0.0, -0.12, 0.20, rx=180.0, ry=0.0, rz=-90.0); rospy.sleep(1)
        self.search_ar_target(bed_number)
        if not self.drl_pose[0] == 0: ## when AR tag is detected, execute the following codes
            self.search_ar_target(bed_number);   movel(self.drl_pose, vel=[130,50], acc=[100,50]);   rospy.sleep(0.5) # 1st approach
            self.search_ar_target(bed_number);   movel(self.drl_pose, vel=[130,50], acc=[100,50]);   rospy.sleep(0.5) # 2nd approach
            self.search_ar_target(bed_number);   movel(self.drl_pose, vel=[130,50], acc=[100,50]);   rospy.sleep(0.5) # 3rd approach
            self.search_ar_target(bed_number);   movel(self.drl_pose, vel=[130,50], acc=[100,50]);   rospy.sleep(0.5) # 4th approach
        else:
            print("ar target not found repeat")
        
        self.search_ar_target(bed_number)
        if not self.drl_pose[0] == 0:
            self.UpdateParam(0.0, 0.0, 0.3, rz=-45.0);   rospy.sleep(1);  self.search_ar_target(bed_number);   waypoint_0 = self.drl_pose
            waypoint_1 = deepcopy(waypoint_0);     waypoint_1[1] -= 120
            waypoint_2 = deepcopy(waypoint_0);     waypoint_2[1] += 35;     waypoint_2[2] -= 100
            waypoint_3 = deepcopy(waypoint_2);     waypoint_3[2] -= 45
            waypoint_4 = P_UNIVERSALJIG_3DP_BED;   waypoint_4[2] += 100
            waypoint_5 = deepcopy(waypoint_4);     waypoint_5[2] -= 100
            
            print("1")
            movel(waypoint_1)
            print("2")
            movel(waypoint_2)
            print("3")
            movel(waypoint_3)
            self.suction_cup_on()
            print("4")
            movel(waypoint_2)
            print("5")
            movel(waypoint_1)
            print("6")
            movel(waypoint_4)
            movel(waypoint_5)
            self.suction_cup_off();  rospy.sleep(1)
            self.jig_x_close();  self.jig_y_close();  rospy.sleep(1)
            movel(waypoint_4)
            movej(Q_TOP_PLATE)

            self.UpdateParam(0.0, -0.12, 0.20, rx=180.0, ry=0.0, rz=-90.0)

    def getBedFromJigToPrinter(self, printer_number):
        bed_number = printer_number + 4
        self.setVelAcc(30, 30, [100,50], [100,50])
        self.jig_x_close();  self.jig_y_close();  rospy.sleep(1)
        movej(Q_TOP_PLATE)

        self.UpdateParam(0.0, -0.12, 0.20, rx=180.0, ry=0.0, rz=-90.0); rospy.sleep(1)
        self.search_ar_target(bed_number)
        if not self.drl_pose[0] == 0: ## when AR tag is detected, execute the following codes
            movej(Q_SEARCH_3DP_PLATE)
            self.movel_z(100)
            self.suction_cup_on();  self.jig_x_open();  self.jig_y_open(); rospy.sleep(1)
            self.movel_z(-100)
            movej(Q_SEARCH_3DP_RIGHT)

            self.search_ar_target(printer_number)
            if not self.drl_pose[0] == 0: ## when AR tag is detected, execute the following codes
                self.search_ar_target(printer_number);  movel(self.drl_pose, vel=[130,50], acc=[100,50]);  rospy.sleep(0.5) # 1st approach
                self.search_ar_target(printer_number);  movel(self.drl_pose, vel=[130,50], acc=[100,50]);  rospy.sleep(0.5) # 2nd approach
                self.search_ar_target(printer_number);  movel(self.drl_pose, vel=[130,50], acc=[100,50]);  rospy.sleep(0.5) # 3rd approach
                self.search_ar_target(printer_number);  movel(self.drl_pose, vel=[130,50], acc=[100,50]);  rospy.sleep(0.5) # 4th approach
            else:
                print("ar target not found repeat")
            
            self.search_ar_target(printer_number)
            if not self.drl_pose[0] == 0:
                self.UpdateParam(-0.09, -0.30, 0.40, rz=135.0);    rospy.sleep(1);  self.search_ar_target(printer_number) 
                waypoint_1 = self.drl_pose;           waypoint_1[0] -= 35
                waypoint_2 = deepcopy(waypoint_1);    waypoint_2[2] -= 90
                waypoint_3 = deepcopy(waypoint_2);    waypoint_3[1] += 295
                waypoint_4 = deepcopy(waypoint_3);    waypoint_4[2] -= 120
                
                movel(waypoint_1, vel=[130,50], acc=[100,50])
                movel(waypoint_2, vel=[130,50], acc=[100,50])
                movel(waypoint_3, vel=[130,50], acc=[100,50])
                movel(waypoint_4, vel=[130,50], acc=[100,50])
                self.suction_cup_off()
                movel(waypoint_3, vel=[130,50], acc=[100,50])
                movel(waypoint_2, vel=[130,50], acc=[100,50])
                movel(waypoint_1, vel=[130,50], acc=[100,50])

                # movej(Q_SEARCH_3DP_PLATE)
                self.UpdateParam(0.0, -0.12, 0.20, rx=180.0, ry=0.0, rz=-90.0)

    
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
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        release_compliance_ctrl()
        self.robot_status = "running"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))
        self.cmd_protocol = int(float(msg.data))
        print(self.cmd_protocol)

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
            self.UpdateParam(0.0, -0.12, 0.20)
            self.search_ar_target(4)
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 1st approach
            self.search_ar_target(4)
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 2nd approach
            self.search_ar_target(4)
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 3rd approach
        # ACTION [5]: ALIGN
        elif(self.cmd_protocol == ACTION_ALIGN):
            self.UpdateParam(0.0, 0.0, 0.2)
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

            P_TOOLCHANGE_1 = [-436.074462890625, -346.8432312011719, 69.4855728149414, 101.02711486816406, 179.40762329101562, 22.690649032592773]
            P_TOOLCHANGE_1 = [-435.8908386230469, -346.9735412597656, 71.24858093261719, 136.8338623046875, 178.95765686035156, 58.61622619628906]
            
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
            self.toolchanger_attach();  rospy.sleep(1)
            movel(p_tool1_step4)
            movel(p_tool1_step5)
        # ACTION [-301]: Tool Changer - Place Tool1 to the Toolchanger1
        elif(self.cmd_protocol == ACTION_TOOLCHANGE_1_DETACH):
            # P_TOOLCHANGE_1 = [-436.074462890625, -346.8432312011719, 69.4855728149414, 101.02711486816406, 179.40762329101562, 22.690649032592773]
            P_TOOLCHANGE_1 = [-435.8908386230469, -346.9735412597656, 71.24858093261719, 136.8338623046875, 178.95765686035156, 58.61622619628906]
            
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
            task_compliance_ctrl([500, 4500, 4000, 1000, 1000, 1000])
            movel(p_tool1_step4)
            release_compliance_ctrl()
            movel(p_tool1_step5);       rospy.sleep(1)
            self.toolchanger_detach();  rospy.sleep(1)
            movel(p_tool1_step8)
            
            movel(p_tool1_step6)

            self.setVelAcc(200, 200, [400,100], [400,100])
            movel(p_tool1_step7)
            
        # ACTION [302]: Tool Changer - Get Tool2 from Toolchanger2
        elif(self.cmd_protocol == ACTION_TOOLCHANGE_2_ATTACH):
            self.toolchanger_detach()

            P_TOOLCHANGE_2 = [-277.7904052734375, -346.1768493652344, 69.29383850097656, 109.82817840576172, 179.61642456054688, 31.086288452148438]
            # P_TOOLCHANGE_2 = [-277.3737487792969, -345.2371826171875, 71.8157958984375, 71.0041732788086, -178.6968231201172, -7.797959804534912]

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
            self.toolchanger_attach();  rospy.sleep(1)
            movel(p_tool2_step4)
            movel(p_tool2_step5)

        # ACTION [-302]: Tool Changer - Place Tool2 to the Toolchanger2
        elif(self.cmd_protocol == ACTION_TOOLCHANGE_2_DETACH):
            # P_TOOLCHANGE_2 = [-277.7904052734375, -346.1768493652344, 69.29383850097656, 109.82817840576172, 179.61642456054688, 31.086288452148438]
            P_TOOLCHANGE_2 = [-277.3737487792969, -345.2371826171875, 71.8157958984375, 71.0041732788086, -178.6968231201172, -7.797959804534912]
            
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
            task_compliance_ctrl([500, 4500, 4000, 1000, 1000, 1000])
            # movel(p_tool2_step4)
            release_compliance_ctrl()
            movel(p_tool2_step5);       rospy.sleep(1)
            self.toolchanger_detach();  rospy.sleep(1)
            movel(p_tool2_step6)

            self.setVelAcc(100, 100, [400,100], [400,100])
            movel(p_tool2_step7)
            self.setVelAcc(50, 50, [50,100], [50,100])
            
            

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
            movej(Q_TOP_PLATE, 50, 50)

            self.setVelAcc(30, 30, [100,50], [100,50])
            see_point1j = [81.08692169189453, -0.4761710464954376, -143.7606658935547, -9.412845611572266, 57.22504806518555, 100.97422790527344]
            movej(see_point1j)

            self.UpdateParam(0.0, -0.12, 0.25)
            rospy.sleep(1)
            self.search_ar_target('4')
            if not self.drl_pose[0] == 0:
                ### Searching for the ar tag#2 which is placed on 3dp door ###
                ## Searching is consisted of three steps
                ## TF: ar_target  -- robot base
                ## AR Target which is a TF made from ar_marker it is facing the marker tf(z axis is towards each other)
                ## Consist of three move which is  conducting scan, move and feedback
                movel(self.drl_pose, vel=[130,50], acc=[100,50]) # 1st approach
                self.search_ar_target('4')
                movel(self.drl_pose, vel=[130,50], acc=[100,50]) # 2nd approach
                self.search_ar_target('4')
                movel(self.drl_pose, vel=[130,50], acc=[100,50]) # 3rd approach
                self.search_ar_target('4')
            else:
                print("ar target not found repeat")
            
            movel([170,70,0,0,0,-90], mod = 1, ref = 1)
            movel([0,0,97,0,0,0], mod = 1, ref = 1)
	    
        # Task [10003]: Place specimen and go to the monitoring position
        elif(self.cmd_protocol == TASK_INSTRON_MOVEOUT):
            rospy.sleep(10)
            self.gripper_open()
            movel([0,0,-200,0,0,0], mod = 1, ref = 1)
            viewpoint = [self.current_posx[0],self.current_posx[1],self.current_posx[2],self.current_posx[3],self.current_posx[4]-20,self.current_posx[5]]
            movel(viewpoint)

        # Task [10004]: Testing compliance mode using scale -> (F = -kx // k=10;10;100 , x=10;10;100)
        elif(self.cmd_protocol == TASK_TEST_COMPLIANCE):
            self.setVelAcc(50, 50, [50,100], [50,100])

            init_posj = Q_BACK
            movej(init_posj)

            self.movel_z(280)
            k = 10
            x = 10
            g = 9.81
            
            eef_weight = self.toolforce[2]
            task_compliance_ctrl([100, 100, 10, 1000, 1000, 1000])


            ##contact point 정의 필요 movel할 것
            while(True):
                if self.toolforce[2] == 0.0:  #set force in N
                    contact_posx = posx(self.current_posx[0], self.current_posx[1], self.current_posx[2]+5, 180, 180, 0)
                    release_compliance_ctrl()
                    print("Z position: {}".format(contact_posx[2]))
                    break

            for k in range(10,1000,10):
                for x in range(10,100,10):
                    task_compliance_ctrl([100, 100, k, 1000, 1000, 1000])
                    self.move_xyz(0, 0, x, velx=[10,10])
                    print("----------------------------------")
                    print("k: {}".format(k), "x: {}".format(x))
                    # print("Expected mass -> {} [g]".format(k*x/g))
                    # print("End-effector mass -> {} [g]".format(-eef_weight/g * 1000))
                    print("Result mass -> {} g".format((self.toolforce_max - eef_weight)/g * 1000))

                    release_compliance_ctrl()
                    movel(contact_posx, vel=[20,20], acc=[100,50])
                    self.toolforce_max = 0.0

        # Task [10005]: Seperate workpiece from the printing bed on the universal jig
        elif(self.cmd_protocol == TASK_SEPARATE_SPECIMEN):
            release_compliance_ctrl()
            self.setVelAcc(50, 50, [50,100], [50,100])
            
            init_posj = [-6.106618404388428, -3.9530131816864014, -111.23585510253906, -3.684967041015625, -69.19756317138672, -1.9469901323318481]
            movej(init_posj,50,50)
            self.movel_z(100)

            k = 100
            
            eef_weight = self.toolforce[2]
            task_compliance_ctrl([100, 100, 10, 1000, 1000, 1000])

            # movel(moving_down,[50,50],[50,50])

            ##contact point 정의 필요 movel할 것
            while(True):
                if self.toolforce[2] == 0.0:  #set force in N
                    contact_posx = posx(self.current_posx[0], self.current_posx[1], self.current_posx[2]+5, 180, 180, 0)
                    release_compliance_ctrl()
                    print("Z position: {}".format(contact_posx[2]))
                    break

            for k in range(10,1000,10):
                for x in range(10,100,10):
                    task_compliance_ctrl([100, 100, k, 1000, 1000, 1000])
                    self.move_xyz(0, 0, x, velx=[10,10])
                    print("----------------------------------")
                    print("k: {}".format(k), "x: {}".format(x))
                    # print("Expected mass -> {} [g]".format(k*x/g))
                    # print("End-effector mass -> {} [g]".format(-eef_weight/g * 1000))
                    print("Result mass -> {} g".format((self.toolforce_max - eef_weight)/g * 1000))

                    release_compliance_ctrl()
                    movel(contact_posx, vel=[20,20], acc=[100,50])
                    self.toolforce_max = 0.0
            pass

        # Task [10006]: SEARCH AND APPROACH TO ''MULTIPLE'' SPECIMENS USING TF
        elif(self.cmd_protocol == TASK_MULSPECIMEN_SEARCH):
            # self.setVelAcc(50, 50, [150,100], [150,100])
            self.setVelAcc(30, 30, [30,30], [30,30])
            # movel([-357.0, 165.0, 322.0, -181.3, -180.0, 0.0])
            movej([-19.77288246154785, 5.743539333343506, -131.46726989746094, -0.0, -54.27621841430664, 161.5270538330078])

            self.gripper_open()
            self.jig_x_open();  self.jig_y_open();  rospy.sleep(2)
            self.jig_x_close(); self.jig_y_close(); rospy.sleep(2)

            object_count=1
            
            ## Publish Flag to 'snu_2d_vision.py' node
            self.vision_pub.publish(VISION_2D_SEARCH_SPECIMEN)
            rospy.sleep(5) #In order to change previous specimen TF

            while True:
                try:
                    target_frame_name = 'specimen_table_' + str(object_count)
                    reference_frame_name = 'base_0'
                    print "Searching specimen ..."
                    print("Target frame: "    + target_frame_name)
                    print("Reference frame: " + reference_frame_name)
                    print "Trying to search the specimen: %s ..."%target_frame_name

                    self.listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(10.0))
                    (trans,rot) = self.listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
                    self.update_cmd_pose(trans, rot)
                    self.updateEulZYZ()
                    self.drl_pose = deepcopy(posx(self.target_pose.position.x, self.target_pose.position.y, 332 , -181.3, -180, -self.eulerZYZ[2]-self.eulerZYZ[0]))
                    print('Target DRL Pose: ' , self.drl_pose)

                    print('search complete')
                    movel(self.drl_pose)
                    self.movel_z(105, [100, 100], [100, 100]) #go down 95 for development
                    self.gripper_close()

                    #SHAKING
                    task_compliance_ctrl([3000, 3000, 3000, 1000, 1000, 1000])
                    # self.movel_x(20, [100, 100], [100, 100])
                    # self.movel_x(-20, [100, 100], [100, 100])
                    self.rotate_z(10)
                    self.rotate_z(-20)
                    self.rotate_z(10)
                    self.movel_y(30)
                    self.rotate_z(10)
                    self.rotate_z(-20)
                    self.rotate_z(10)
                    release_compliance_ctrl()

                    self.movel_z(-105,[100, 100], [100, 100]) #go up
                    movej([-19.77288246154785, 5.743539333343506, -131.46726989746094, -0.0, -54.27621841430664, 161.5270538330078])
                    
                    # specimen_loc = [-403.63006591796875+object_count*50, -104.22643280029297, 186.01812744140625, 37.57080078125, 178.80581665039062, -144.4349365234375]
                    # self.setVelAcc(100, 100, [500,100], [500,100])
                    # movel(specimen_loc)
                    
                    self.gripper_open()
                    # movej([-19.77288246154785, 5.743539333343506, -131.46726989746094, -0.0, -54.27621841430664, 161.5270538330078])
                    object_count= object_count+1

                except (Exception):
                    print "[ERROR]: The Target(TF) is not Detected !!!"
                    print("Specimen count :{}".format(object_count-1))
                    break

            self.jig_x_open();  self.jig_y_open()


        # Task [10007]: IDIM block demo (under development...)
        elif(self.cmd_protocol == TASK_IDIM_BLOCK_DEMO):
            self.setVelAcc(50, 50, [50,100], [50,100])
            search_init_pos = [-20.134538650512695, 15.13610553741455, -125.55142211914062, -0.0, -69.58480072021484, -20.13439178466797+180]
            movej(search_init_pos)
            
            #Set Region of Interest
            rowEnd=565
            colEnd=345

            rowStart=239 #224
            colStart=20

            #Offset from camera to endeffector
            cam_offsetx = 116
            cam_offsety = 43


            bgr_temp = self.specimen_image
            gray_temp=cv2.cvtColor(bgr_temp, cv2.COLOR_BGR2GRAY)
            
            bgr=bgr_temp[colStart:colEnd, rowStart:rowEnd]
            gray=gray_temp[colStart:colEnd, rowStart:rowEnd]

            edges=cv2.Canny(gray,50,200)
            # cv2.imshow('Canny', edges)
            cv2.waitKey(0)
            _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # cv2.imshow('Contours',contours)
            # cv2.waitKey(0)
            # box_pose = np.array([])
            box_pose = []
            try:
                for ii in range(len(contours)):
                    ptAccum=np.squeeze(contours[ii])

                    # FILTER1 : the specimen edge should contain more than 300 points
                    if len(ptAccum) <1: 
                        print('bad search : point shortage')
                        print(len(ptAccum))
                        continue

                    x_Max=np.argmax(ptAccum[:,0]) # x maximum coordinate index
                    x_Min=np.argmin(ptAccum[:,0]) # x minimum coordinate index
                    y_Max=np.argmax(ptAccum[:,1]) # y maximum coordinate index
                    y_Min=np.argmin(ptAccum[:,1]) # y minimum coordinate index

                    #find four rectnagular Vertices using maximum coordinate
                    x_Max_Vertice=[ptAccum[x_Max,0], ptAccum[x_Max,1]]
                    x_Min_Vertice=[ptAccum[x_Min,0], ptAccum[x_Min,1]]
                    y_Max_Vertice=[ptAccum[y_Max,0], ptAccum[y_Max,1]]
                    y_Min_Vertice=[ptAccum[y_Min,0], ptAccum[y_Min,1]]

                    # FILTER2
                    print(ptAccum[x_Max,0], ptAccum[x_Min,0], ptAccum[y_Max,1], ptAccum[y_Min,1])            

                    orientation1= float(x_Max_Vertice[1]-x_Min_Vertice[1])/float(x_Max_Vertice[0]-x_Min_Vertice[0])
                    orientation2= float(y_Max_Vertice[1]-y_Min_Vertice[1])/float(y_Max_Vertice[0]-y_Min_Vertice[0])
                    orientation=(orientation1+orientation2)/2.0
                    theta=math.atan(orientation)

                    #centroid : average of all coordinates
                    centroid=[float(sum(ptAccum[:,0])/float(len(ptAccum[:,0]))), float(sum(ptAccum[:,1]))/float(len(ptAccum[:,1]))]
                    # box_pix = gray[ptAccum[x_Min,0]:ptAccum[x_Max,0],ptAccum[y_Min,0]:ptAccum[y_Max,0]]
                    box_pix = ptAccum[x_Min:x_Max,y_Min:y_Max]
                    text = pytesseract.image_to_string(rgb,lang='eng')
                    # print('the text is' + text)
                    # cv2.imshow('box_pix',box_pix)
                    # cv2.waitKey()
            except:
                print "[ERROR]"
                # box_pose_temp = [centroid[0],centroid[1]]
                #plotting for debugging 
                # cv2.circle(bgr, (int(centroid[0]), int(centroid[1])),2,(0,0,255),4)
                # cv2.circle(bgr, (int(y_Max_Vertice[0]), int(y_Max_Vertice[1])),1,(0,255,255),2)
                # cv2.circle(bgr, (int(x_Min_Vertice[0]), int(x_Min_Vertice[1])),1,(0,255,255),2)
                # cv2.circle(bgr, (int(y_Min_Vertice[0]), int(y_Min_Vertice[1])),1,(0,255,255),2)
                # cv2.circle(bgr, (int(x_Max_Vertice[0]), int(x_Max_Vertice[1])),1,(0,255,255),2)
                # cv2.imshow('image', bgr)
                # text = pytesseract.image_to_string(img,lang='euc')
                

        # Task [10008]: JOG DEVELOPMENT
        elif(self.cmd_protocol == TASK_JOG_DEVEL):

            initial_pick_pose = [-357.0, 165.0, 322.0, -180.0, -180.0, 0.0]
            incline_pick_pose = [-357.0, 165.0, 322.0, 90.0, -135.0, 0.0]
            
            initial_place_pose = [-277, -100.0, 322.0, -180.0, -180.0, 0.0]
            incline_place_pose = [-277, -100.0, 180.0, 90.0, 135.0, 0.0]

            lack_one = [-277.0, 168.5, 166.0, 90.0, -135.0, 0.0]
            lack_two = copy.deepcopy(lack_one); lack_two[1] -= 23.55; 
            lack_three = copy.deepcopy(lack_one); lack_three[1] -= 23.55 * 2.0; 
            lack_four = copy.deepcopy(lack_one); lack_four[1] -= 23.55 * 3.0; 

            lack_place_one = [-277.0, -80.0, 172.0, 90.0, 135.0, 0.0]
            lack_place_two = copy.deepcopy(lack_place_one); lack_place_two[1] -= 23.55; 
            lack_place_three = copy.deepcopy(lack_place_one); lack_place_three[1] -= 23.55 * 2.0; 
            lack_place_four = copy.deepcopy(lack_place_one); lack_place_four[1] -= 23.55 * 3.0; 

            self.setVelAcc(50, 50, [150,100], [150,100])
            # movel([-357.0, 165.0, 322.0, -181.3, -180.0, 0.0])
            movej([-19.77288246154785, 5.743539333343506, -131.46726989746094, -0.0, -54.27621841430664, 161.5270538330078])

            self.gripper_open()
            self.jig_x_open();  self.jig_y_open();  rospy.sleep(5)
            self.jig_x_close(); self.jig_y_close(); rospy.sleep(3)

            object_count=1
            
            ## Publish Flag to 'snu_2d_vision.py' node
            self.vision_pub.publish(VISION_2D_SEARCH_SPECIMEN)
            rospy.sleep(5) #In order to change previous specimen TF

            while True:
                try:
                    target_frame_name = 'specimen_table_' + str(object_count)
                    reference_frame_name = 'base_0'
                    print "Searching specimen ..."
                    print("Target frame: "    + target_frame_name)
                    print("Reference frame: " + reference_frame_name)
                    print "Trying to search the specimen: %s ..."%target_frame_name

                    self.listener.waitForTransform(reference_frame_name, target_frame_name, rospy.Time(), rospy.Duration(10.0))
                    (trans,rot) = self.listener.lookupTransform(reference_frame_name, target_frame_name, rospy.Time(0))
                    self.update_cmd_pose(trans, rot)
                    self.updateEulZYZ()
                    self.drl_pose = deepcopy(posx(self.target_pose.position.x, self.target_pose.position.y, 332 , -181.3, -180, -self.eulerZYZ[2]-self.eulerZYZ[0]))
                    print('Target DRL Pose: ' , self.drl_pose)

                    print('search complete')
                    movel(self.drl_pose)
                    self.movel_z(105, [100, 100], [100, 100]) #go down 95 for development
                    self.gripper_close()
                    self.movel_z(-105,[100, 100], [100, 100]) #go up
                    movej([-19.77288246154785, 5.743539333343506, -131.46726989746094, -0.0, -54.27621841430664, 161.5270538330078])
                    

                    self.setVelAcc(30, 30, [30,30], [30,30])

                    movel(initial_pick_pose)

                    movel(initial_place_pose)
                    movel(incline_place_pose)

                    movel(lack_place_one)
                    self.move_lack_place()

                    movel(incline_pick_pose)

                    movel(lack_one)
                    self.move_lack_pick()
                    movel(initial_pick_pose)

                    object_count= object_count+1

                except (Exception):
                    print "[ERROR]: The Target(TF) is not Detected !!!"
                    print("Specimen count :{}".format(object_count-1))
                    break

            self.jig_x_open();  self.jig_y_open()
            
            # movel(lack_place_two)
            # self.move_lack_place()
            
            # movel(lack_place_three)
            # self.move_lack_place()

            # movel(lack_place_four)
            # self.move_lack_place()

            # movel(incline_pick_pose)

            # movel(lack_one)
            # self.move_lack_pick()
            # self.gripper_open()

            # movel(lack_two)
            # self.move_lack_pick()
            # self.gripper_open()

            # movel(lack_three)
            # self.move_lack_pick()
            # self.gripper_open()

            # movel(lack_four)
            # self.move_lack_pick()
            # self.gripper_open()

        # Task [10011]: "TASK_3DP_1_BED_IN"   - (3DP-#1 Bed) Jig -> Printer 
        elif(self.cmd_protocol == TASK_3DP_1_BED_IN):
            self.getBedFromJigToPrinter(1)
        # Task [-10011]: "TASK_3DP_1_BED_OUT" - (3DP-#1 Bed) Printer -> Jig 
        elif(self.cmd_protocol == TASK_3DP_1_BED_OUT):
            self.getBedFromPrinterToJig(1)

        # Task [10012]: "TASK_3DP_2_BED_IN"   - (3DP-#2 Bed) Jig -> Printer 
        elif(self.cmd_protocol == TASK_3DP_2_BED_IN):
            self.getBedFromJigToPrinter(2)
        # Task [-10012]: "TASK_3DP_2_BED_OUT" - (3DP-#2 Bed) Printer -> Jig 
        elif(self.cmd_protocol == TASK_3DP_2_BED_OUT):
            self.getBedFromPrinterToJig(2)

        # Task [10013]: "TASK_3DP_3_BED_IN"   - (3DP-#3 Bed) Jig -> Printer 
        elif(self.cmd_protocol == TASK_3DP_3_BED_IN):
            self.getBedFromJigToPrinter(3)
        # Task [-10013]: "TASK_3DP_3_BED_OUT" - (3DP-#3 Bed) Printer -> Jig 
        elif(self.cmd_protocol == TASK_3DP_3_BED_OUT):
            self.getBedFromPrinterToJig(3)

        # Task [10014]: "TASK_3DP_4_BED_IN"   - (3DP-#4 Bed) Jig -> Printer 
        elif(self.cmd_protocol == TASK_3DP_4_BED_IN):
            self.getBedFromJigToPrinter(4)
        # Task [-10014]: "TASK_3DP_4_BED_OUT" - (3DP-#4 Bed) Printer -> Jig 
        elif(self.cmd_protocol == TASK_3DP_4_BED_OUT):
            self.getBedFromPrinterToJig(4)



        ########################################################################################################################################################
        set_robot_mode(ROBOT_MODE_MANUAL)
        release_compliance_ctrl()  
        self.robot_status = "done"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))
    
        

if __name__=='__main__':
    idim = DRLInterface("snu_drl_commander")
    
    while not rospy.is_shutdown():
        pass
        if(idim.dsr_flag == 2):
            idim.robot_status = "running"
        #elif(idim.dsr_flag == 1 and idim.robot_status == "running"):
        #    idim.robot_status = "done"
        #else:
        #    idim.robot_status = "waiting"
        idim.status_pub.publish(URStatus(status=idim.robot_status, arm_status = idim.joints_state))
        rospy.sleep(0.1)
    # set_robot_mode(ROBOT_MODE_MANUAL)
    
