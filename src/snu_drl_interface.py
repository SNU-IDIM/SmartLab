#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_IDIM_ASMR/common/imp"%HOME_DIR)) )
from IDIM_header import *
from IDIM_framework import *

ROS_NODE_NAME = "snu_drl_commander"

SUB_TOPIC_1 = "ur_pnp"
SUB_TOPIC_2 = "dsr/state"
SUB_TOPIC_3 = "dsr/joint_states"

PUB_TOPIC_1 = "ur_pnp"
PUB_TOPIC_2 = "ur_status"


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
    def __init__(self):
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

        rospy.init_node(ROS_NODE_NAME, anonymous=True)
        self.listener = tf.TransformListener()
        rospy.Subscriber(SUB_TOPIC_1, String, self.pnp_cb, queue_size=1)
        rospy.Subscriber(SUB_TOPIC_2, RobotState, self.dsr_state_cb, queue_size=1)
        rospy.Subscriber(SUB_TOPIC_3 ,JointState, self.current_status_cb, queue_size=1)
        self.pnp_pub    = rospy.Publisher(PUB_TOPIC_1, String, queue_size=1)
        self.status_pub = rospy.Publisher(PUB_TOPIC_2, URStatus, queue_size=1)

        
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        rospy.sleep(1)
        self.robot_status = "working"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))
        movej(Q_TOP_PLATE, 50, 50)
        self.robot_status = "done"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))


    #'''
    #    update_cmd_pose: update 'target_pose' to feed 'movel' function for Doosan-robot
    #        @ input 1: geometry_msgs/Vector3 trans
    #        @ input 2: geometry_msgs/Quaternion rot
    #'''
    def update_cmd_pose(self, trans, rot):
        self.target_pose.position.x    = M2MM(trans[0])# + self.offset_x) # 보정 @(arm -> 측면)
        self.target_pose.position.y    = M2MM(trans[1])# + self.offset_y) # 보정 @(arm -> 정면)
        self.target_pose.position.z    = M2MM(trans[2])# + self.offset_z)
        self.target_pose.orientation.x = rot[0]
        self.target_pose.orientation.y = rot[1]
        self.target_pose.orientation.z = rot[2]
        self.target_pose.orientation.w = rot[3]
        print(self.target_pose)

    #'''
    #    updateEulZYZ: Calculate ZYZ rotation to feed 'movel' function for Doosan-robot
    #'''
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

    #'''
    #    search_ar_target: lookupTransform to get AR_Target (calculated from AR_Marker)
    #        @ input 1: int ar_tag_number (ex - 0, 1, 2, 3, ...)
    #'''
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

    #'''
    #    UpdateParam: Updating parameters for target pose w.r.t. AR_Marker
    #        @ input 1: double dx [m]
    #        @ input 2: double dy [m]
    #        @ input 3: double dz [m]
    #'''
    def UpdateParam(self, dx, dy, dz):
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/x', dx)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/y', dy)
        rospy.set_param('/R_001/snu_object_tracker/offset_from_target/z', dz)
        rospy.sleep(2)
    

    #'''
    #    Doosan-robot Relative Move (translation in x, y, z [mm])
    #        @ input 1: double distance [mm]
    #'''
    def movel_x(self, distance): # distance [mm]
        movel(posx(distance, 0, 0, 0, 0, 0), vel=[50,10], acc=[50,10], ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_y(self, distance): # distance [mm]
        movel(posx(0, distance, 0, 0, 0, 0), vel=[50,10], acc=[50,10], ref=DR_TOOL, mod=DR_MV_MOD_REL)
    def movel_z(self, distance): # distance [mm]
        movel(posx(0, 0, distance, 0, 0, 0), vel=[50,10], acc=[50,10], ref=DR_TOOL, mod=DR_MV_MOD_REL)


    #'''
    #    dsr_state_cb: "~/dsr/state" topic callback function (update dsr_flag)
    #'''
    def dsr_state_cb(self, data):
        self.dsr_flag = data.robot_state


    #'''
    #    current_status_cb: update "~/ur_status" from "~/dsr/joint_state"
    #'''
    def current_status_cb(self, data):
        self.joints_state = data


    #'''
    #    "~/ur_pnp" Topic Protocol (for Doosan-robot control)
    #    
    #    Naming rules:
    #        @ 정의된 숫자 순서에 맞는 위치에 정의 (오름차순)
    #        @ ACTION_[이름 정의(대문자)] : 0.0  ~ 10000.0
    #            *    0.0 ~  100.0: Basic Move
    #            *  101.0 ~  200.0: Doosan-robot I/O Controller
    #            * 1000.0 ~ 4000.0: Relative Move (Translation)
    #                - X -> 1000.0 (0 mm) ~ 1999.0 (999 mm)
    #                - Y -> 2000.0 (0 mm) ~ 2999.0 (999 mm)
    #                - Z -> 3000.0 (0 mm) ~ 3999.0 (999 mm)
    #        @ TASK_[이름 정의(대문자)]   : 10001.0  ~ 20000.0
    #'''
    def pnp_cb(self, msg):
        self.robot_status = "running"
        self.cmd_protocol = msg.data
        print(msg.data)

        # ACTION (0.0): Home position
        if(self.cmd_protocol   == ACTION_HOME):         
            movej(Q_HOME, 50, 50)
        # ACTION (1.0): Back position
        elif(self.cmd_protocol == ACTION_BACK):
            movej(Q_BACK, 50, 50)
        # ACTION (2.0): Left position
        elif(self.cmd_protocol == ACTION_LEFT):
            movej(Q_LEFT, 50, 50)
        # ACTION (3.0): Right position
        elif(self.cmd_protocol == ACTION_RIGHT):
            movej(Q_RIGHT, 50, 50)
        # ACTION (4.0): Approach
        elif(self.cmd_protocol == ACTION_APPROACH):
            movej(Q_TOP_PLATE, 50, 50) # Search pose
            self.UpdateParam(0.0, -0.12, 0.20)
            self.search_ar_target(1)
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 1st approach
            self.search_ar_target(1)
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 2nd approach
            self.search_ar_target(1)
            movel(self.drl_pose, vel=[100,50], acc=[100,50]) # 3rd approach
        # ACTION (5.0): Approach  
        elif(self.cmd_protocol == ACTION_ALIGN):
            self.UpdateParam(0.0, 0.0, 0.2)
            self.search_ar_target(1)
            movel(self.drl_pose, vel=[100,30], acc=[100,30])
        # ACTION (6.0): Pick
        elif(self.cmd_protocol == ACTION_PICK):
            self.pnp_pub.publish(ACTION_IO_GRIPPER_OPEN)
            self.movel_z(30)
            self.pnp_pub.publish(ACTION_IO_GRIPPER_CLOSE)
            self.movel_z(-30)
        # ACTION (7.0): Place
        elif(self.cmd_protocol == ACTION_PLACE):
            self.pnp_pub.publish(ACTION_IO_GRIPPER_CLOSE)
            self.movel_z(30)
            self.pnp_pub.publish(ACTION_IO_GRIPPER_OPEN)
            self.movel_z(-30)
        # ACTION (1000.0 ~ 1999.0): Trans X (relative move)
        elif(abs(int(float(self.cmd_protocol))) >= int(float(ACTION_TRANS_X)) and abs(int(float(self.cmd_protocol))) < int(float(ACTION_TRANS_Y)) ):
            sign = int(float(self.cmd_protocol)) / abs(int(float(self.cmd_protocol)))
            self.movel_x(sign * (abs(int(float(self.cmd_protocol))) - int(float(ACTION_TRANS_X))))
        # ACTION (2000.0 ~ 2999.0): Trans Y (relative move)
        elif(abs(int(float(self.cmd_protocol))) >= int(float(ACTION_TRANS_Y)) and abs(int(float(self.cmd_protocol))) < int(float(ACTION_TRANS_Z)) ):
            sign = int(float(self.cmd_protocol)) / abs(int(float(self.cmd_protocol)))
            self.movel_y(sign * (abs(int(float(self.cmd_protocol))) - int(float(ACTION_TRANS_Y))))
        # ACTION (3000.0 ~ 4000.0): Trans Z (relative move)
        elif(abs(int(float(self.cmd_protocol))) >= int(float(ACTION_TRANS_Z)) and abs(int(float(self.cmd_protocol))) < int(float(ACTION_TRANS)) ):
            sign = int(float(self.cmd_protocol)) / abs(int(float(self.cmd_protocol)))
            self.movel_z(sign * (abs(int(float(self.cmd_protocol))) - int(float(ACTION_TRANS_Z))))

        self.robot_status = "done"
        self.status_pub.publish(URStatus(status=self.robot_status, arm_status = self.joints_state))
    
    def test(self):
        self.pnp_pub.publish(ACTION_IO_GRIPPER_OPEN)
        # self.pnp_pub.publish(ACTION_IO_GRIPPER_CLOSE)
        

if __name__=='__main__':
    idim = DRLInterface()
    idim.test()
    
    while not rospy.is_shutdown():
        if(idim.dsr_flag == 2):
            idim.robot_status = "running"
        elif(idim.robot_status == "running" and idim.dsr_flag == 1):
            idim.robot_status = "done"
        else:
            idim.robot_status = "waiting"
        idim.status_pub.publish(URStatus(status=idim.robot_status, arm_status = idim.joints_state))
        rospy.sleep(0.1)
    