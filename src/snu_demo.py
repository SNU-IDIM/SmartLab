#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys, numpy as np
import moveit_commander
import timeit
from math import pi
from time import sleep
from copy import deepcopy

import moveit_msgs.msg
from std_msgs.msg import String, Header
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from syscon_msgs.msg import URStatus
from dsr_msgs.msg import RobotState
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import *

# Some of Contstants
DISTANCE_AWAY_FROM_TARGET = 0.2

EPSILON = 0.0000001
DEG2RAD = 3.141592 / 180.0

ROLL = 180.0 * DEG2RAD
PITCH = 0.0 * DEG2RAD
YAW = 0.0 * DEG2RAD



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

class MoveGroupPythonInteface(object):
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('m1013_demo', anonymous=True)

    group_name = "arm"
    reference_frame = "/base_link"

    self.joints_state = None
    self.robotus = 'waiting'
    self.dsr_flag = None
    self.start_pose = Pose()
    self.target_pose = Pose()
    self.artag = AlvarMarkers()
    
    self.dsr_status = rospy.Publisher('ur_status', URStatus, queue_size=1)

    rospy.Subscriber('ur_pnp', String, self.dsr_PickPlace_cb, queue_size=1)
    rospy.Subscriber('dsr_moveit', Pose, self.dsr_moveit_cb, queue_size=1)
    rospy.Subscriber('dsr/state', RobotState, self.dsr_state_cb, queue_size=1)
    rospy.Subscriber('dsr/joint_states',JointState, self.current_status_cb, queue_size=1)

    self.robot = moveit_commander.RobotCommander(robot_description="dsr/robot_description", ns="dsr") # outer-level interface to the robot
    self.scene = moveit_commander.PlanningSceneInterface(ns="dsr") # world surrounding the robot
    self.group = moveit_commander.MoveGroupCommander(group_name, robot_description="dsr/robot_description", ns="dsr") # interface to one group of joints (plan & execute)
    #self.gruop.set_pose_reference_frame(reference_frame)
    
    Q0 = [0.0, 0.0, -90.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0]
    #self.default_joint_states = self.group.get_current_joint_values()
    self.default_joint_states = Q0

    # Allow replanning to increase the odds of a solution
    self.group.allow_replanning(True) 
    self.group.set_goal_position_tolerance(0.01)
    self.group.set_goal_orientation_tolerance(0.1)
    self.group.set_planning_time(0.1)
    self.group.set_max_acceleration_scaling_factor(0.5)
    self.group.set_max_velocity_scaling_factor(0.65)

    # We can also print the name of the end-effector link for this group:
    self.end_effector_link = self.group.get_end_effector_link()
   # print "============ End effector: %s" % self.end_effector_link

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()

   # print "============ Moveit_Commander Setup Initialized !!!"
   # print "Robot Pose Initialized!"
    self.group.set_joint_value_target(self.default_joint_states)
    self.group.set_start_state_to_current_state()
    plan = self.group.plan()
    self.robotus = "running"
    self.dsr_status.publish(URStatus(status=self.robotus, arm_status = self.joints_state))
    self.group.execute(plan)
    self.robotus = "done"
    self.dsr_status.publish(URStatus(status=self.robotus, arm_status = self.joints_state))
    self.robotus = "waiting"

##############################################################################################################
  def dsr_state_cb(self, data):
    self.dsr_flag = data.robot_state

  def current_status_cb(self, data):
	  self.joints_state = data

  def dsr_moveit_cb(self,msg):
    self.start_pose  = self.group.get_current_pose(self.end_effector_link).pose
    self.target_pose = self.start_pose

    self.target_pose.position.x += msg.position.x
    self.target_pose.position.y += msg.position.y
    self.target_pose.position.z += msg.position.z
    




    q1 = []
    q1.append(self.target_pose.orientation.x)
    q1.append(self.target_pose.orientation.y)
    q1.append(self.target_pose.orientation.z)
    q1.append(self.target_pose.orientation.w)
    
    q2 = []
    q2.append(msg.orientation.x)
    q2.append(msg.orientation.y)
    q2.append(msg.orientation.z)
    q2.append(msg.orientation.w)

    q3 = [] 
    q3 = quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))
    print'1111111111111111111111111111111111111111111111111111'
    print(q3)
    self.target_pose.orientation.x = q3[0]
    self.target_pose.orientation.y = q3[1]
    self.target_pose.orientation.z = q3[2]
    self.target_pose.orientation.w = q3[3]

    print'2222222222222222222222222222222222222222222222222222'
    print(self.target_pose.orientation)
    
    waypoints = []
    waypoints.append(self.start_pose)
    waypoints.append(self.target_pose)

    print(waypoints)
    
    #print(target_pose.position.x)
    #target_pose.position.x    = start_pose.position.x + 0.1
    #target_pose.position.y    = start_pose.position.y
    #target_pose.position.z    = start_pose.position.z
    #target_pose.orientation.x = msg.orientation.x
    #target_pose.orientation.y = msg.orientation.y
    #target_pose.orientation.z = msg.orientation.z
    #target_pose.orientation.w = msg.orientation.w
    #print(target_pose)
    
    
    #waypoints.append(deepcopy(target_pose))
    
    
    #print 'start pose: %s'%start_pose
    #print 'target pose :%s'%target_pose
    #print 'waypoints :%s'%waypoints
    self.group.set_start_state_to_current_state()
    plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
    if 1-fraction < 0.2:
       self.group.execute(plan)
    

  def dsr_PickPlace_cb(self,msg):
    print(msg.data)
    self.start_flag = msg.data 
    if(self.start_flag=="1.0"):
      Q2 = [90*DEG2RAD, 0.0, -90*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0]
      self.default_joint_states = Q2
      self.group.set_joint_value_target(self.default_joint_states)
      self.group.set_start_state_to_current_state()
      plan = self.group.plan()
      self.group.execute(plan)
      self.start_flag="0.0"

    if(self.start_flag=="-1.0"):
      Q1 = [-90*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0]
      self.default_joint_states = Q1
      self.group.set_joint_value_target(self.default_joint_states)
      self.group.set_start_state_to_current_state()
      plan = self.group.plan()
      self.group.execute(plan)
      self.start_flag="0.0"

    if(self.start_flag=="0.0"):
      Q0 = [0.0, 0.0, -90.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0]
      self.default_joint_states = Q0
      self.group.set_joint_value_target(self.default_joint_states)
      self.group.set_start_state_to_current_state()
      plan = self.group.plan()
      self.group.execute(plan)

    if(self.start_flag=="10.0"):

        
        self.start_pose = self.group.get_current_pose(self.end_effector_link).pose
        self.waypoints.append(deepcopy(self.start_pose))
        print(self.start_pose)
        
        self.target_pose = self.artag.markers[idx].pose.position.x 
        #print(target_pose.position.x)
        target_pose.position.x    = start_pose.position.x + 0.1
        #target_pose.position.y    = start_pose.position.y
        #target_pose.position.z    = start_pose.position.z
        #target_pose.orientation.x = msg.orientation.x
        #target_pose.orientation.y = msg.orientation.y
        #target_pose.orientation.z = msg.orientation.z
        #target_pose.orientation.w = msg.orientation.w
        print(target_pose)
        
        
        waypoints.append(deepcopy(target_pose))
        
        
        #print 'start pose: %s'%start_pose
        #print 'target pose :%s'%target_pose
        #print 'waypoints :%s'%waypoints
        self.group.set_start_state_to_current_state()
        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
        if 1-fraction < 0.2:
           self.group.execute(plan)



'''
  def dsr_moveit_cb(self,msg):
        print(msg)
        waypoints = []
        
        start_pose = self.group.get_current_pose(self.end_effector_link).pose
        waypoints.append(deepcopy(start_pose))
        print(start_pose)
        
        # target_pose = start_pose
        #print(target_pose.position.x)
        target_pose.position.x    = start_pose.position.x + 100
        #target_pose.position.y    = start_pose.position.y
        #target_pose.position.z    = start_pose.position.z
        #target_pose.orientation.x = msg.orientation.x
        #target_pose.orientation.y = msg.orientation.y
        #target_pose.orientation.z = msg.orientation.z
        #target_pose.orientation.w = msg.orientation.w
        print(target_pose)
        
        
        waypoints.append(deepcopy(target_pose))
        
        
        #print 'start pose: %s'%start_pose
        #print 'target pose :%s'%target_pose
        #print 'waypoints :%s'%waypoints
        self.group.set_start_state_to_current_state()
        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
        if 1-fraction < 0.2:
           self.group.execute(plan)
'''
##############################################################################################################

if __name__=='__main__':
  mp=MoveGroupPythonInteface()
  while not rospy.is_shutdown():
    if(mp.dsr_flag == 2):
      mp.robotus = "running"
    elif(mp.robotus == "running" and mp.dsr_flag == 1):
      mp.robotus = "done"
    else:
			mp.robotus = "waiting"
    mp.dsr_status.publish(URStatus(status=mp.robotus, arm_status = mp.joints_state))
    rospy.sleep(0.1)