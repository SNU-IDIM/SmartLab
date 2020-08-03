#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_IDIM_ASMR/common/imp"%HOME_DIR)) )
from IDIM_header import *
from IDIM_framework import *


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
    def UpdateParam(self, dx, dy, dz):
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/x', dx)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/y', dy)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/z', dz)
        rospy.sleep(2)
    

    '''
        Doosan-robot Relative Move (translation in x, y, z [mm])
            @ input 1: double distance [mm]
            @ input 2: intArray velx = [50, 10] [mm/s, mm/s]
            @ input 3: intArray accx = [50, 10] [mm/s^2, mm/s^2]
    '''
    def movel_x(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm]
        movel(posx(distance, 0, 0, 0, 0, 0), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_y(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm]
        movel(posx(0, distance, 0, 0, 0, 0), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_z(self, distance, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm]
        movel(posx(0, 0, distance, 0, 0, 0), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def move_xyz(self, x, y, z, velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm]
        movel(posx(x, y, z, 0, 0, 0), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_xyzjoint(self,x,y,z,jz1,jy,jz2,velx=DSR_DEFAULT_JOG_VELX, accx=DSR_DEFAULT_JOG_ACCX): # distance [mm], angle [degree]
        movel(posx(x, y, z, jz1, jy, jz2), vel=velx, acc=accx, ref=DR_TOOL, mod=DR_MV_MOD_REL)


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
        DSR I/O Functions
            1. Gripper open/close
            2. Compressor on/off
            3. Tool Changer attach/detach
    '''
    def IO_init(self):
        for i in range(1, 16+1):
            if i == 6 or i ==5:
                pass
            else:
                set_digital_output(i, 0)
            
        rospy.sleep(0.5)
    def gripper_close(self):
        self.IO_init();  set_digital_output(13,1)
    def gripper_open(self):
        self.IO_init();  set_digital_output(14,1)
    def compressor_on(self):
        self.IO_init();  set_digital_output(1,1)
    def compressor_off(self):
        self.IO_init();  set_digital_output(1,0)
    def toolchanger_attach(self):
        self.IO_init();  set_digital_output(2,0)
    def toolchanger_detach(self):
        self.IO_init();  set_digital_output(2,1)





    
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
        self.robot_status = "running"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))
        self.cmd_protocol = int(float(msg.data))
        print(self.cmd_protocol)

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
        # ACTION [5]: Approach  
        elif(self.cmd_protocol == ACTION_ALIGN):
            self.UpdateParam(0.0, 0.0, 0.2)
            self.search_ar_target(1)
            movel(self.drl_pose, vel=[100,30], acc=[100,30])

        # ACTION [101]: Gripper Open
        elif(self.cmd_protocol == ACTION_IO_GRIPPER_OPEN):
            self.gripper_open()
        # ACTION [102]: Gripper Close
        elif(self.cmd_protocol == ACTION_IO_GRIPPER_CLOSE):
            self.gripper_close()
        # ACTION [103]: Compressor On
        elif(self.cmd_protocol == ACTION_IO_COMPRESSOR_ON):
            self.compressor_on()
        # ACTION [104]: Compressor Off
        elif(self.cmd_protocol == ACTION_IO_COMPRESSOR_OFF):
            self.compressor_off()
        # ACTION [105]: Tool-changer Attach
        elif(self.cmd_protocol == ACTION_IO_TOOLCHANGER_ATTACH):
            self.toolchanger_attach()
        # ACTION [106]: Tool-changer Detach
        elif(self.cmd_protocol == ACTION_IO_TOOLCHANGER_DETACH):
            self.toolchanger_detach()

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
            movej(Q_TOP_PLATE, 50, 50)

            self.gripper_open()

            set_velj(50); set_accj(50); set_velx(100,50); set_accx(100,150)
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
            
            set_velj(self.robvelj); set_accj(self.robaccj); set_velx(self.robvelx+50,self.robvelx); set_accx(self.robaccx+50,self.robaccx)
            see_point1j = [81.08692169189453, -0.4761710464954376, -143.7606658935547, -9.412845611572266, 57.22504806518555, 100.97422790527344]
            movej(see_point1j,self.robvelj,self.robaccj)

            self.UpdateParam(0.0, -0.12, 0.25)
            rospy.sleep(1)
            self.search_ar_target('4')
            if not self.drl_pose[0] == 0:
                ########searching for the ar tag#2 which is placed on 3dp door####
                #Searching is consisted of three steps
                #TF: ar_target  -- robot base
                # ar target which is a TF made from ar_marker it is facing the marker tf(z axis is towards each other)
                #consist of three move which is  conducting scan, move and feedback
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
        
        # Task [10004]: Seperate workpiece from the printing bed on the universal jig
        elif(self.cmd_protocol == TASK_SEPARATE):
            release_compliance_ctrl()

            set_velj(50)
            set_accj(50)
            set_velx(50,100)
            set_accx(50,100)
            
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
        
        # Task [20001]
        elif(self.cmd_protocol == TASK_TEST_COMPLIANCE):
            release_compliance_ctrl()

            set_velj(50)
            set_accj(50)
            set_velx(50,100)
            set_accx(50,100)
            
            # init_posj = [-6.106618404388428, -3.9530131816864014, -111.23585510253906, -3.684967041015625, -69.19756317138672, -1.9469901323318481]
            init_posj = Q_BACK
            movej(init_posj,50,50)

            self.movel_z(280)
            k = 10
            x = 10
            g = 9.81
            
            
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

        # Task [10006]: SEARCH AND APPROACH TO ''SINGLE'' SPECIMEN
        elif(self.cmd_protocol == TASK_SPECIMEN_SEARCH):
            set_velj(50)
            set_accj(50)
            set_velx(50,100)
            set_accx(50,100)
            set_digital_output(8,1)
            set_digital_output(9,1)
            
            # init1 = [-389.43310546875, 216.06378173828125, 310.4500427246094, 0, 180, 0]
            # movel(init1)
            # init2 = [-389.43310546875, 266.06378173828125, 310.4500427246094, 0, 180, 0]
            # movel(init2)

            #low height joint angle
            # search_init_pos = [-24.57923698425293, -0.10278947651386261, -127.49447631835938, -0.0, -52.4029426574707, -24.57932472229004] 

            #high height joint angle
            search_init_pos = [-20.134538650512695, 15.13610553741455, -125.55142211914062, -0.0, -69.58480072021484, -20.13439178466797]
            movej(search_init_pos)

            # cv2.imshow('image',self.specimen_image)
            # cv2.waitKey(30000)

            while(True):
                try:
                    #set Region of Interest
                    # DEPTH=210

                    rowEnd=607
                    colEnd=383
                    rowStart=224
                    colStart=20

                    cam_offsetx = 116
                    cam_offsety = 43
                    
                    bgr_temp = self.specimen_image
                    gray_temp=cv2.cvtColor(bgr_temp, cv2.COLOR_BGR2GRAY)

                    gray=gray_temp[colStart:colEnd, rowStart:rowEnd]
                    bgr=bgr_temp[colStart:colEnd, rowStart:rowEnd]
                    
                    #Canny edge detection & Hough lines transform
                    edges=cv2.Canny(gray,50,200)
                   
                    lines=cv2.HoughLines(edges, 1, np.pi/180, 15, None, 0, 0)

                    print(lines)
                    # cv2.imshow('Hough', lines)
                    # cv2.waitKey(10) 
                    
                    recAccum=[]
                    ptAccum=[]
                    for i in range(0, len(lines)):
                        for j in range(i+1,len(lines)):
                            if abs(abs(lines[i][0][1]-lines[j][0][1])-np.pi/2)<0.02:
                                recAccum.append([i,j])

                    if recAccum is not None:
                        for i in range(0,len(recAccum)):
                            num1=recAccum[i][0]
                            rho1 = lines[num1][0][0]
                            theta1 = lines[num1][0][1]
                            if rho1 <0:
                                rho1= abs(rho1)
                                theta1=theta1+np.pi
                            a1 = math.cos(theta1)
                            b1 = math.sin(theta1)
                            x01 = a1 * rho1
                            y01 = b1 * rho1

                            num2=recAccum[i][1]
                            rho2 = lines[num2][0][0]
                            theta2 = lines[num2][0][1]
                            if rho2 <0:
                                rho2= abs(rho2)
                                theta2=theta2+np.pi
                            a2 = math.cos(theta2)
                            b2 = math.sin(theta2)
                            x02 = a2 * rho2
                            y02 = b2 * rho2

                            if b2==0:
                                x=rho2
                                y=rho1
                            elif b1==0:
                                x=rho1
                                y=rho2
                            else:
                                x=(1-(b1/b2)*(rho2/rho1))*rho1/(a1-a2*b1/b2)
                                y=(1-a1*x/rho1)*rho1/b1
                            print(x,y)
                            ptAccum.append([x,y])
                            cv2.circle(bgr, (int(x),int(y)),5,(0,255,255),2)
                    

                    ptAccum=np.array(ptAccum)
                    print(ptAccum)
                    x_Max=np.argmax(ptAccum[:,0]) # x maximum coordinate index
                    x_Min=np.argmin(ptAccum[:,0]) # x minimum coordinate index
                    y_Max=np.argmax(ptAccum[:,1]) # y maximum coordinate index
                    y_Min=np.argmin(ptAccum[:,1]) # y minimum coordinate index

                    y_Max_Vertice=[ptAccum[y_Max,0], ptAccum[y_Max,1]]
                    x_Max_Vertice=[ptAccum[x_Max,0], ptAccum[x_Max,1]]
                    x_Min_Vertice=[ptAccum[x_Min,0], ptAccum[x_Min,1]]
                    y_Min_Vertice=[ptAccum[y_Min,0], ptAccum[y_Min,1]]

                    orientation1= (x_Max_Vertice[1]-x_Min_Vertice[1])/(x_Max_Vertice[0]-x_Min_Vertice[0])
                    orientation2= (y_Max_Vertice[1]-y_Min_Vertice[1])/(y_Max_Vertice[0]-y_Min_Vertice[0])
                    orientation=(orientation1+orientation2)/2

                    theta=math.atan(orientation)
                    centroid=[(np.max(ptAccum[:,0])+np.min(ptAccum[:,0]))/2, (np.max(ptAccum[:,1])+ np.min(ptAccum[:,1]))/2]
                
                    print('centroid',centroid)
                    print('lt',y_Max_Vertice)
                    print('lb',x_Min_Vertice)
                    print('rt',x_Max_Vertice)
                    print('rb',y_Min_Vertice)
                    print(theta*180/np.pi)
                    print(orientation1)
                    print(orientation2)
                    cv2.circle(bgr, (int(centroid[0]), int(centroid[1])),2,(0,0,255),4)
                    cv2.circle(bgr, (int(y_Max_Vertice[0]), int(y_Max_Vertice[1])),1,(0,0,255),2)
                    cv2.circle(bgr, (int(x_Min_Vertice[0]), int(x_Min_Vertice[1])),1,(0,0,255),2)
                    cv2.circle(bgr, (int(y_Min_Vertice[0]), int(y_Min_Vertice[1])),1,(0,0,255),2)
                    cv2.circle(bgr, (int(x_Max_Vertice[0]), int(x_Max_Vertice[1])),1,(0,0,255),2)

                    px2mm_Row=(centroid[0]+rowStart-320)*100/188
                    px2mm_Col=(centroid[1]+colStart-240)*100/188

                    print(px2mm_Row, px2mm_Col)


                    cv2.imshow('image', bgr)
                    cv2.waitKey(10)

                    if centroid is not None:
                        print('search complete')
                        # search_pos = [-389.43310546875-cam_offsetx+px2mm_Col, 216.06378173828125-cam_offsety+px2mm_Row, 310.4500427246094, 0, 180, -90+theta*180/np.pi]
                        
                        self.movel_xyzjoint(cam_offsetx-px2mm_Col+10, -cam_offsety+px2mm_Row,0,0,0,-90+theta*180/np.pi, [100, 100], [100, 100])
                        self.movel_z(200, [100, 100], [100, 100]) #go down
    
                        break

                    # self.update_ros_param()
                    # self.static_transformStamped.header.stamp    = rospy.Time.now()
                    # # self.static_transformStampeed.header.frame_id = self.object_frame_name
                    # # self.static_transformStamped.child_frame_id  = self.target_frame_name

                    # self.static_transformStamped.transform.translation.x = px2mm_Row
                    # self.static_transformStamped.transform.translation.y = px2mm_Col*(-1.0)
                    # self.static_transformStamped.transform.translation.z = 0.0

                    # quat = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
                    # self.static_transformStamped.transform.rotation.x = quat[0]
                    # self.static_transformStamped.transform.rotation.y = quat[1]
                    # self.static_transformStamped.transform.rotation.z = quat[2]
                    # self.static_transformStamped.transform.rotation.w = quat[3]
                    # #print "%s"%self.static_transformStamped
                
                    # self.broadcaster.sendTransform(self.static_transformStamped)

                except:
                    print('keep searching')
                    continue

        # Task [10007]: SEARCH AND APPROACH TO ''MULTIPLE'' SPECIMENS
        elif(self.cmd_protocol == TASK_MULSPECIMEN_SEARCH):
            set_velj(50)
            set_accj(50)
            set_velx(50,100)
            set_accx(50,100)
            search_init_pos = [-20.134538650512695, 15.13610553741455, -125.55142211914062, -0.0, -69.58480072021484, -20.13439178466797]
            movej(search_init_pos)
            set_digital_output(13,0)
            set_digital_output(14,1)
            
            set_digital_output(5,0)
            set_digital_output(6,0)
            
            rospy.sleep(5)
            set_digital_output(5,1)
            set_digital_output(6,1)
            

            rospy.sleep(3)
            
            # the same position with below joint position
            # init1=[-328.0, 157.0, 435.0, 0, 180, 0]
            # movel(init1)

            

            #Set Region of Interest
            rowEnd=600
            colEnd=400
            rowStart=220 #224
            colStart=20

            #Offset from camera to endeffector
            cam_offsetx = 116
            cam_offsety = 43

            bgr_temp = self.specimen_image
            gray_temp=cv2.cvtColor(bgr_temp, cv2.COLOR_BGR2GRAY)

            gray=gray_temp[colStart:colEnd, rowStart:rowEnd]
            bgr=bgr_temp[colStart:colEnd, rowStart:rowEnd]

            #Canny edge detection & Hough lines transform
            edges=cv2.Canny(gray,50,200)
            cv2.imshow('Canny', edges)
            cv2.waitKey(1000)
            _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
            for ii in range(len(contours)):
                ptAccum=np.squeeze(contours[ii])

                # FILTER1 : the specimen edge should contain more than 300 points
                if len(ptAccum) <500: 
                    print('bad search : point shortage')
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

                #plotting for debugging 
                cv2.circle(bgr, (int(centroid[0]), int(centroid[1])),2,(0,0,255),4)
                cv2.circle(bgr, (int(y_Max_Vertice[0]), int(y_Max_Vertice[1])),1,(0,255,255),2)
                cv2.circle(bgr, (int(x_Min_Vertice[0]), int(x_Min_Vertice[1])),1,(0,255,255),2)
                cv2.circle(bgr, (int(y_Min_Vertice[0]), int(y_Min_Vertice[1])),1,(0,255,255),2)
                cv2.circle(bgr, (int(x_Max_Vertice[0]), int(x_Max_Vertice[1])),1,(0,255,255),2)
                cv2.imshow('image', bgr)
                cv2.waitKey(30)

                #Calibration 100mm / 188pixels
                px2mm_Row=(centroid[0]+rowStart-320)*100/188
                px2mm_Col=(centroid[1]+colStart-240)*100/188

                if centroid is not None:
                    print('search complete')
                    # search_pos = [-389.43310546875-cam_offsetx+px2mm_Col, 216.06378173828125-cam_offsety+px2mm_Row, 310.4500427246094, 0, 180, -90+theta*180/np.pi]
                    self.movel_xyzjoint(cam_offsetx-px2mm_Col+10, -cam_offsety+px2mm_Row,0,0,0,theta*180/np.pi, [100, 100], [100, 100])
                    self.movel_z(204, [100, 100], [100, 100]) #go down
                    self.gripper_close()
                    self.movel_z(-204,[100, 100], [100, 100]) #go up
                    movej(search_init_pos)
                    self.gripper_open()
                    # set_digital_output(8,0)
                    # set_digital_output(9,0)
                    set_digital_output(5,0)
                    set_digital_output(6,0)

        elif(self.cmd_protocol == TOOLCHANGE1):
            release_compliance_ctrl()
            
            

            set_velj(50)
            set_accj(50)
            set_velx(150,100)
            set_accx(150,100)
            # tool2_position_ = [-236.8995361328125, -358.78912353515625, 38.94343566894531, 82.39966583251953, 178.2489013671875, 2.303342580795288]
            # movel(tool2_position_)
            home = [-235.78550720214844, -358.7340393066406, 352.00294494628906, 84.49280548095703, 177.22393798828125, 4.466612815856934]
            movel(home)
            # approach = [-235.78550720214844, -358.7340393066406, 252.00294494628906, 84.49280548095703, 177.22393798828125, 4.466612815856934]
            # movel(approach)
            approach = [-235.98329162597656, -358.2447509765625, 49.16860580444336, 82.3934097290039, 177.25328063964844, 2.3672542572021484]
            movel(approach)


            set_velx(50,100)
            set_accx(50,100)
            task_compliance_ctrl([500, 4500, 100, 1000, 1000, 1000])


            approach1 = [-236.38015747070312, -358.78912353515625, 39.48764419555664, 84.1766586303711, 177.68301391601562, 4.175314426422119]
            movel(approach1)

            approach2 = [-236.38015747070312, -358.78912353515625+40, 39.48764419555664, 84.1766586303711, 177.68301391601562, 4.175314426422119]
            movel(approach2)

            # approach3 = [-236.38015747070312, -358.78912353515625, 39.48764419555664, 84.1766586303711, 177.68301391601562, 4.175314426422119]
            # movel(approach3)
            release_compliance_ctrl()

            tool2_position = [-236.3540496826172, -336.87152099609375, 38.58928298950195, 168.34994506835938, 179.523681640625, 88.14895629882812]
            movel(tool2_position)




            # set_stiff
            # print(release_compliance_ctrl())
            # print(task_compliance_ctrl([100, 100, 100, 100, 100, 100]))
            # rospy.sleep(3)
            set_digital_output(2,1)
            rospy.sleep(1)

            # release_compliance_ctrl()
            # task_compliance_ctrl([1000, 1000, 10000, 200, 200, 200])
            self.movel_z(-100)
            tool1_position = [-375.4557800292969, -337.03533935546875, 38.78656005859375+100, 121.61463165283203, 179.27223205566406, 41.801002502441406]
            movel(tool1_position)
            
            tool1_position = [-375.4557800292969, -337.03533935546875, 38.78656005859375, 121.61463165283203, 179.27223205566406, 41.801002502441406]
            movel(tool1_position)

            set_digital_output(2,0)

            move_out1 = [-375.4557800292969, -337.03533935546875-20, 38.78656005859375, 121.61463165283203, 179.27223205566406, 41.801002502441406]
            movel(move_out1)

            move_out2 = [-375.4557800292969, -337.03533935546875-25, 38.78656005859375+250, 121.61463165283203, 179.27223205566406, 41.801002502441406]
            movel(move_out2)
            

            # moveout =[-236.8995361328125, -337.1667175292969, 38.94343566894531+100, 82.39966583251953, 178.2489013671875, 2.303342580795288]
            # movel(moveout)
            # release_compliance_ctrl()
            






            

            # home[1] = home[1]-80
            # movel(home)

        set_robot_mode(ROBOT_MODE_MANUAL)        
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
    
