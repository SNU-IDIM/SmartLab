#! /usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi

import rospy
import actionlib
import tf, PyKDL
from syscon_msgs.msg import *
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../../snu_idim_common/imp")) )
# sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_SmartLAB/snu_idim_common/imp"%HOME_DIR)) )
from IDIM_framework import *

            
class StateMachine():
    '''
        __init__: Initializer
            @input 1: ros_node_name - Specifying the node name (default: "snu_state_machine")
            @input 2: work_id - Set default work id (default: "")
            @input 3: loop_number - How many time to repeat the task (default: 1)
    '''
    def __init__(self, ros_node_name="snu_state_machine", work_id="default", loop_number=1):
        self.goal = WorkFlowGoal()
        self.goal.work = []
        self.goal.work_id = work_id # default: "snu_state_machine"
        self.goal.loop_flag = 1     # default: 1 (no repeat)
        self.dsr_action = String()

        self.dsr_status = None
        self.dsr_flag   = 0
        self.amr_param = [Param('max_trans_vel','float','0.3'),
                          Param('max_rot_vel','float','0.25'), 
                          Param('xy_goal_tolerance','float','0.20'),
                          Param('yaw_goal_tolerance','float','0.05')]

        self.jetson_status = None

        rospy.init_node(ros_node_name, anonymous=True)
        self.client = actionlib.SimpleActionClient('/R_001/WAS', WorkFlowAction)
        self.client.wait_for_server(timeout=rospy.Duration(1))
        self.pnp_pub = rospy.Publisher("/R_001/ur_pnp", String, queue_size=1)
        self.jetson_pub = rospy.Publisher("/pc_to_jetson", Float64, queue_size=1)
        rospy.Subscriber("/R_001/ur_status", URStatus, self.dsr_status_cb, queue_size=1)
        rospy.Subscriber("/jetson", String, self.jetson_cb, queue_size=1)
        rospy.sleep(3.0)

        print("-------------------------------------")
        print("  IDIM State Machine Initialized !!!")
        print("    - Work ID: {}".format(work_id))
        print("    - Repeat: {} times".format(loop_number))
        print("-------------------------------------")


    '''
        dsr_status_cb: Get current status of Doosan-robot ("waiting" / "working" / "done")
    '''
    def dsr_status_cb(self, msg):
        if(self.dsr_status != msg.status):
            self.dsr_flag = 1
            self.dsr_status = msg.status
            print("[Cobot] DSR Status Changed !!!")
            # print(self.dsr_status)
        else:
            self.dsr_flag = 0

    def jetson_cb(self, msg):
        self.jetson_status = msg.data

    '''
        addWorkAMR: add AMR Work to the Task
            @input 1: target_pose - target pose for AMR (x, y, theta) [m, rad]
    '''
    def addWorkAMR(self, target_pose):
        work = Action(SYSCON_WAYPOINT, target_pose, self.amr_param)
        self.goal.work.append(work)


    '''
        addWorkDSR: add DSR Work to the Task
            @input 1: action_name - Action name for DSR (defined in IDIM_framework.py)
    '''
    def addWorkDSR(self, action_name):
        work = Action(SYSCON_URMISSION, [SYSCON_URPNP, int(float(action_name))],[])
        self.goal.work.append(work)

    '''
        executeAMR
    '''
    def executeAMR(self, target_pose, work_name="default", hold_time=0.0):
        print("[AMR] AMR Start Moving ... (target pose = {})".format(target_pose))
        work = Action(SYSCON_WAYPOINT, target_pose, self.amr_param)
        self.goal.work.append(work)
        self.goal.work_id = work_name
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        print("[AMR] Arrived at [{}] !!!".format(target_pose))
        self.goal.work = []
        rospy.sleep(hold_time)

    '''
        executeDSR
    '''
    def executeDSR(self, action_name, hold_time=1.0):
        self.dsr_action = str(action_name)
        self.pnp_pub.publish(self.dsr_action)
        self.waitforDSR(action_name, hold_time)
    

    def JetsonComm(self, action_name, hold_time=1.0):
        self.jetson_pub.publish(action_name)
        rospy.sleep(2)
        self.waitforJetson(action_name, hold_time)

    def waitforJetson(self, action_name, hold_time):
        while(1):
            print('[Instron] Experiment Running ...')
            if self.jetson_status == 'done':
                break
        print("[Instron] Experiment({}) Completed !!!".format(action_name))
        rospy.sleep(hold_time)

    

    '''
        execute: Execute the task & clear the work list
            @input 1: work_id - name of the task (could be anything)
    '''
    def execute(self, work_id="Default_Work_ID"):
        self.goal.work_id = work_id
        print(self.goal)
        self.client.send_goal(self.goal)
        # self.client.wait_for_result()
        # self.waitforDSR()
        self.goal = WorkFlowGoal()
        print("{} DONE !!!".format(self.goal))
        

    def waitforDSR(self, action_name, hold_time):
        while(1):
            if self.dsr_flag == 1 and self.dsr_status == 'done':
                break
        print("[Cobot] DSR WORK [{}] Done !!!".format(action_name))
        rospy.sleep(hold_time)
    


'''
    사용법
        1. StateMachine 인스턴스 생성
        2. executeAMR / executeDSR 로 work 추가 (sequence에 맞게 실행)
'''
if __name__ == '__main__':
    sm = StateMachine(work_id="IDIM_State_Machine", loop_number=1)

    # # sm.executeAMR(work_name='Instron', target_pose=[3.016, -3.244, -1.622], hold_time=0.0)
    # # sm.executeDSR(TASK_SPECIMEN_PICK)
    # # sm.executeDSR(TASK_INSTRON_SEARCH)
    # # sm.JetsonComm(10)
    # # sm.executeDSR(TASK_INSTRON_MOVEOUT, 3)
    # # sm.executeDSR(ACTION_HOME)
    # # sm.JetsonComm(20)
    # # sm.JetsonComm(30)

    # sm.executeDSR(ACTION_TOOLCHANGE_1_DETACH)
    # sm.executeDSR(ACTION_TOOLCHANGE_2_ATTACH)
    # # sm.executeDSR(TASK_MULSPECIMEN_SEARCH)
    # sm.executeDSR(ACTION_TOOLCHANGE_2_DETACH)
    # sm.executeDSR(ACTION_HOME)





    ## Smart lab routine
    # sm.executeDSR(ACTION_HOME)                  # 1. Home position에서 시작
    # sm.executeDSR(ACTION_TOOLCHANGE_1_ATTACH)   # 2. Tool-1 장착 (Suction-cup)
    # sm.executeDSR(TASK_3DP_3_BED_OUT)           # 3. 3DP-3 프링팅 베드 이동: 프린터 -> 로봇
    # sm.executeDSR(ACTION_HOME)                  # 4. Home position 이동

    # sm.executeDSR(ACTION_TOOLCHANGE_1_DETACH)   # 5. Tool-1 해체
    # sm.executeDSR(ACTION_TOOLCHANGE_2_ATTACH)   # 6. Tool-2 장착 (Parallel gripper)
    # sm.executeDSR(ACTION_HOME)                  # 7. Home position 이동
    # sm.executeDSR(TASK_DETACH_SPECIMEN)         # 8. 프린팅 베드에서 시편 분리
    # sm.executeDSR(TASK_SEARCH_PICK_SPECIMEN)    # 9. 분리된 시편 pick
    # sm.executeDSR(TASK_SEPCIMEN_TO_CENTER)      # 10. 시편 이동: 작업 공간 중간

    # sm.executeDSR(TASK_ADHESIVE_SAVER_OUT)      # 11. Adhesive 마개 제거
    # sm.executeDSR(TASK_SPECIMEN_TO_LEFT)        # 12. 시편 이동: 작업 공간 왼쪽
    # sm.executeDSR(TASK_ADHESIVE_DROP)           # 13. 시편에 adhesive drop
    # sm.executeDSR(TASK_SPECIMEN_TO_RIGHT)       # 14. 시편 이동: 작업 공간 오른쪽
    # sm.executeDSR(TASK_ADHESIVE_SAVER_IN)       # 15. Adhesive 마개 채결
    # sm.executeDSR(TASK_ATTACH_SENSOR + 1)       # 16. Color sensor 부착

    # sm.executeDSR(TASK_SPECIMEN_READY)          # 17. 센서가 부착된 시편 이동: 작업 공간 중간
    # sm.executeDSR(TASK_PICK_PLACE_RACK)         # 18. 센서가 부착된 시편 이동: specimen rack
    # sm.executeDSR(TASK_RACK_ALIGN)              # 19. Specimen rack 정렬
    
    # sm.executeDSR(ACTION_TOOLCHANGE_2_DETACH)   # 20. Tool-2 해체
    # sm.executeDSR(ACTION_TOOLCHANGE_1_ATTACH)   # 21. Tool-1 장착 (Suction-cup)
    # sm.executeDSR(TASK_3DP_3_BED_IN)            # 22. 3DP 프링팅 베드 이동: 로봇 -> 프린터
    # sm.executeDSR(ACTION_HOME)                  # 23. Home position 이동

    sm.executeDSR(TASK_INSTRON_SEARCH)          # 24. Instron 장비 referencing
    sm.JetsonComm(10.0)                         # 25. Experiment start
    rospy.sleep(3.0)
    sm.executeDSR(TASK_INSTRON_MOVEOUT)         # 26. Gripper open & 실험 장면 촬영
    sm.JetsonComm(20.0)
    sm.executeDSR(ACTION_HOME)
    # rospy.sleep(10.0)
    # sm.JetsonComm(20.0)


    
    '''
    while True:
        ## specimen feeding test
        sm.executeDSR(TASK_PICK_PLACE_RACK_TEST)
        sm.executeDSR(TASK_INSTRON_SEARCH)
        sm.JetsonComm(10.0)
        rospy.sleep(5.0)
        sm.executeDSR(TASK_INSTRON_MOVEOUT)
        sm.JetsonComm(20.0)
        rospy.sleep(2.0)
    '''


