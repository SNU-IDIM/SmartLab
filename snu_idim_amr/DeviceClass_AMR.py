#!/usr/bin/env python

import os, sys
import json
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from syscon_msgs.msg import *
from threading import Thread
from time import sleep
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/imp")) )
from IDIM_header import *
from IDIM_framework import *


class DeviceClass_AMR:

	def __init__(self, device_name='R_001/amr'):
		## Common init for all devices
		self.status = dict()
		self.status['device_type'] = 'AMR'
		self.status['device_name'] = device_name
		self.status['connection'] = False;	  self.flag_connection = False
		self.status['subject_name'] = ''
		self.status['status'] = ''
		self.status['recent_work'] = ''

		## for AMR command
		self.client = actionlib.SimpleActionClient('/R_001/WAS', WorkFlowAction)
		self.client.wait_for_server(timeout=rospy.Duration(1))
		self.amr = WorkFlowGoal()
		self.amr_param = [Param('max_trans_vel','float','0.3'),
							Param('max_rot_vel','float','0.25'), 
							Param('xy_goal_tolerance','float','0.20'),
							Param('yaw_goal_tolerance','float','0.05')]
		self.amr.work = []
		self.amr.work_id = ''
		self.amr.loop_flag = 1  # default: 1 (no repeat)

		# rospy.Subscriber("/R_001/move_base/status", GoalStatusArray, self.move_base_cb, queue_size=1)
		rospy.Subscriber("/R_001/robot_state", SysconRobotState, self.amr_state_cb, queue_size=1)
		self.status_pub = rospy.Publisher("amr/status", String, queue_size=1)

	
	def __del__(self):
		print('[DEBUG] Node is terminated !!!')
		
	def publishStatus(self):
		msg_json = json.dumps(self.status)
		self.status_pub.publish(msg_json)


	def command(self, cmd_dict):
		target_pose = cmd_dict['target_pose']
		spot_name   = cmd_dict['spot_name']
		hold_time   = cmd_dict['hold_time']
		
		work = Action(SYSCON_WAYPOINT, target_pose, self.amr_param)
		self.amr.work.append(work)
		self.amr.work_id = spot_name
		self.client.send_goal(self.amr)
		self.client.wait_for_result()
		self.amr.work = []
		sleep(hold_time)

		
	def amr_state_cb(self, msg):
		if self.flag_connection == False:
			self.status['connection'] = True
			self.flag_connection = True

		self.status['pose'] = {'x': msg.pose.x,
							   'y': msg.pose.y,
							   'theta': msg.pose.theta,}

		self.status['status'] = msg.workstate
		self.status['current_work'] = None
		self.status['recent_work'] = None

		msg_json = json.dumps(self.status)
		self.status_pub.publish(msg_json)
	

	# def move_base_cb(self, msg):
	# 	print(msg)


if __name__=='__main__':

	rospy.init_node('DeviceClass_AMR')
	amr = DeviceClass_AMR()
	time.sleep(3.0)
	cmd_dict = {
		'spot_name': 'instron', 
		'target_pose': AMR_POS_INSTRON, 
		'hold_time': 0.0,
	}
	
	amr.command(cmd_dict)

	while True:
		amr.publishStatus()
		time.sleep(1.0)
		pass