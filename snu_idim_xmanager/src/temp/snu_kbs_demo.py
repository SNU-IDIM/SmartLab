#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib

from syscon_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import tf, PyKDL
from math import pi

# Action Modes (SYSCON 정의 Action Mode -> 수정 불가)
WAYPOINT  = 0x01
URMOVEIT  = 0x02
URPNP     = 0x03
URMISSION = 0x05

# 사용법-> Action(URMISSION, [URPNP, '정의한 string'])
PICK  =  1.0
PLACE = -1.0
DEMO = 'demo'

# Target Pose Lists [ X[m], Y[m], Theta[rad] ]

DIR_UP    = 0.0
DIR_DOWN  = pi
DIR_LEFT  = -pi/2
DIR_RIGHT = pi/2


P_HOME      = [0.0,             0.0,               0.0          ]
P_TASK_1    = [0.961866204884,  -2.55481989557,    1.52572908801]
P_TASK_2    = [-0.75452822448,  -0.0418339422644,  -0.102579635089]


dsr_status = 'idle' # 두산로봇 상태 정의: 'idle', 'moving'

def dsr_statue_cb(msg):
    global dsr_status
    dsr_status = msg.status
    print(dsr_status)


if __name__ == '__main__':
    global dsr_status

    rospy.init_node('snu_kbs_demo')
    #rospy.Subscriber('/R_001/ur_status', URStatus, dsr_statue_cb, queue_size=1)

    client = actionlib.SimpleActionClient('R_001/WAS', WorkFlowAction)
    client.wait_for_server(timeout=rospy.Duration(1))

    goal = WorkFlowGoal()


    #URMOVEIT = 0x02; URPNP = 0x03
    #PICK = 1.0; PLACE = -1.0

    p = [Param('max_trans_vel','float','0.3'), Param('max_rot_vel','float','0.25'), Param('xy_goal_tolerance','float','0.20'),Param('yaw_goal_tolerance','float','0.05')]
    

    #arm_cmd = Action(URMISSION, [URPNP, PICK],[])
    wp0 = Action(WAYPOINT, P_HOME,   p)
    wp1 = Action(WAYPOINT, P_TASK_1, p)
    wp2 = Action(WAYPOINT, P_TASK_2, p)


    arm_task1 = Action(URMISSION, [URPNP, 1.0],[])
    arm_task2 = Action(URMISSION, [URPNP, 2.0],[])




    
    goal.work_id = "SNU_TEST"; goal.loop_flag = 1
    print"stat"
    goal.work = [arm_task1]
    client.send_goal(goal)
    print 'fin'

    print "START -> TASK_1: 3D Printer"
    goal.work = [wp1, arm_task1]
    client.send_goal(goal)
    print(dsr_status)

    client.wait_for_result()
    print(dsr_status)
    while dsr_status == 'idle':
        print(dsr_status)
        break

    print "START -> TASK_2: CNC"
    goal.work = [wp2, arm_task2]
    client.send_goal(goal)
    client.wait_for_result()
    while(1):
        if dsr_status == 'working':
            print("DSR Status: Working")
            pass
        elif dsr_status == 'idle':
            print("DSR Status: Idle")
            break

    print "START -> TASK_3: Homing"





    '''
    sm.setActionDSR()
    
    # wp1 = Action(SYSCON_WAYPOINT,  P_AMR_HOME,     p)
    # wp2 = Action(SYSCON_WAYPOINT,  P_AMR_TABLE_2,  p)
    # wp3 = Action(SYSCON_WAYPOINT,  P_AMR_TABLE_3,  p)

    # arm_left  = Action(SYSCON_URMISSION, [SYSCON_URPNP, int(float(SYSCON_LEFT))],[])
    # arm_right = Action(SYSCON_URMISSION, [SYSCON_URPNP, int(float(SYSCON_RIGHT))],[])

    # arm_cmd   = Action(SYSCON_URMISSION, [SYSCON_URPNP, 2.0],[])
    # arm_pick  = Action(SYSCON_URMISSION, [SYSCON_URPNP, 3.0],[])
    # arm_place = Action(SYSCON_URMISSION, [SYSCON_URPNP, 4.0],[])


    # goal.work = [arm_left]
    # print(goal)
    # client.send_goal(goal)
    # client.wait_for_result()

    # while dsr_status == 'running':
    #     print(dsr_status)
    #     break

    # goal.work = [arm_right]
    # print(goal)
    # client.send_goal(goal)
    # client.wait_for_result()

    # while dsr_status == 'running':
    #     print(dsr_status)
    #     break



    # rospy.sleep(35)

    # goal.work = [wp3, arm_place]

    # client.send_goal(goal)
    # client.wait_for_result()
    # rospy.sleep(35)
    
    # goal.work = [wp2]

    # client.send_goal(goal)
    # client.wait_for_result()
    '''