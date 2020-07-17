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

        self.offset_x = 0.0
        self.offset_y = -0.12
        self.offset_z = 0.2
        self.robvelj = 30
        self.robaccj = 30
        self.robvelx = 50
        self.robaccx = 50
        
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
        vision_cb: OpenCV image visualization
    '''
    def vision_cb(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.draw_image = copy.deepcopy(self.cv_image) # move this when you draw something in the image and change imagewindowflag
        except CvBridgeError as e:
            print(e)
        if self.imagewindowflag ==0:
            cv2.namedWindow('robot endeffector image', cv2.WINDOW_NORMAL)
            cv2.imshow('robot endeffector image', self.cv_image)
            cv2.waitKey(1)
        elif self.imagewindowflag ==1:
            cv2.namedWindow('robot endeffector image', cv2.WINDOW_NORMAL)
            cv2.imshow('robot endeffector image', self.draw_image)
            cv2.waitKey(1)


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
    '''
    def movel_x(self, distance): # distance [mm]
        movel(posx(distance, 0, 0, 0, 0, 0), vel=[50,10], acc=[50,10], ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_y(self, distance): # distance [mm]
        movel(posx(0, distance, 0, 0, 0, 0), vel=[50,10], acc=[50,10], ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_z(self, distance): # distance [mm]
        movel(posx(0, 0, distance, 0, 0, 0), vel=[50,10], acc=[50,10], ref=DR_TOOL, mod=DR_MV_MOD_REL)


    '''
        DSR I/O Functions
            1. Gripper open/close
            2. Compressor on/off
            3. Tool Changer attach/detach
    '''
    def IO_init(self):
        for i in range(1, 16+1):
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
        dsr_state_cb: "~/dsr/state" topic callback function (update dsr_flag)
    '''
    def dsr_state_cb(self, data):
        self.dsr_flag = data.robot_state
        self.current_posx = data.current_posx


    '''
        current_status_cb: update "~/ur_status" from "~/dsr/joint_state"
    '''
    def current_status_cb(self, data):
        self.joints_state = data

    
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
    set_robot_mode(ROBOT_MODE_MANUAL)
    
