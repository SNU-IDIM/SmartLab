#!/usr/bin/env python

import time
import serial
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import String
import numpy as np
import json
import threading




class UART:
	def __init__(self, port_="/dev/ttyTHS1", baudrate_=115200):
		self.serial = serial.Serial(port=port_,
									baudrate=baudrate_,
									bytesize=serial.EIGHTBITS,
									parity=serial.PARITY_NONE,
									stopbits=serial.STOPBITS_ONE)
		self.serial.flushInput()
		self.continue_flag = -1
		self.goal_status = ''

		self.status = dict()
		self.status['subject_name'] = 'None'
		self.status['connection'] = 'offline'
		self.status['status'] = 'waiting'

		self.newstatus = dict()
		

	def changeflag(self,var):
		
		if self.status['status'] == var :
			self.continue_flag = 1

		else:
			self.continue_flag = -1


	def write_data(self, data):
		data = json.dumps(data)
		self.serial.write(data.encode())


	def updateStatus(self):
		while True:
			print(self.status)

			if self.serial.inWaiting() > 0:
				self.serial.flushInput()
				time.sleep(0.2)

				self.data = self.serial.readline().split('\n')#.decode('utf-8',errors = 'replace')
				self.data = self.serial.readline().decode('utf-8',errors = 'replace').split('\n')[0]
				self.newstatus = json.loads(self.data)
				self.status.update(self.newstatus)
				# print("1")

			else:
				# print('Waiting for Serial \n')
				continue



	def waitStatus(self):
		while True:
			self.changeflag(self.goal_status)
			# print(self.status)
			if self.continue_flag == 1:
				self.continue_flag = -1
				print('[Status] {}'.format(self.status))

				break
			else:
				# print("Waiting for the next step\n")
				continue


class Jetson:
	def __init__(self):
		self.PIN = 15
		
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.PIN, GPIO.OUT, initial=GPIO.LOW)

		GPIO.setwarnings(False)

		rospy.init_node('jetson_node')
		rospy.Subscriber('instron/command', String, self.cmd_instron)

		self.instron_status_pub = rospy.Publisher("instron/status", String, queue_size=1)

		self.uart = UART()
		thread = threading.Thread(target = self.uart.updateStatus)
		thread.daemon=True
		thread.start()

		self.command = dict()
		self.command['setup'] = ''                                     # command from server (setup/execute)
		self.command['execute'] = ''

		self.message = dict()												# message from Jetson to Instron
		self.message['message'] = 'online'
		self.message['subject_name'] = ''

		data = ""

		self.uart.write_data(self.message)                                    # check online to Instron
		time.sleep(1)
		self.uart.goal_status = 'Idle'										# wait until status = Idle
		# print(self.uart.status)
		self.uart.waitStatus()											# status : Idle

	#----------------------------------------------Debugging!!!-----------------------------------
		rospy.Subscriber('debugging_cmd', String, self.Debugging)
		self.pub = rospy.Publisher('instron/command',String,queue_size =1)

		self.debug_cmd = dict()
		self.debug_cmd['setup'] = 'specimen_dh'
		self.debug_cmd2 = dict()
		self.debug_cmd2['execute'] = 'specimen_dh'
		# self.debug_cmd = self.debug_cmd2
		# print(self.debug_cmd)

	def Debugging(self, msg):
		if str(msg.data) == '1':
			self.debug_cmd = json.dumps(self.debug_cmd)
			time.sleep(0.5)
			print(self.debug_cmd)
			self.pub.publish(self.debug_cmd)
		elif str(msg.data) == '2':
			self.debug_cmd2 = json.dumps(self.debug_cmd2)
			time.sleep(0.5)
			print(self.debug_cmd2)
			self.pub.publish(self.debug_cmd2)
	#------------------------------------------------------------------------------------------------

	
	def __del__(self):
		GPIO.cleanup()
		print('[DEBUG] Node is terminated !!!')


	def pub_status(self, status):
		status = str(status)
		self.instron_status_pub.publish(status)


	def cmd_instron(self, msg):
		self.instron_cmd = str(msg.data)
		self.command = json.loads(self.instron_cmd)
		time.sleep(0.5)
		print(self.command)
		self.cmd_keys = self.command.keys()
		self.cmd_values = self.command.values()

		print('[DEBUG] Instron command: {}'.format(self.command))
		for i in range(len(self.cmd_keys)):
			if self.cmd_keys[i] == "setup":                         			# experiment setup trigger
				self.uart.status['subject_name'] = self.cmd_values[i]
				self.message['subject_name'] = self.cmd_values[i]
				self.message['message'] = 'start'
				self.uart.write_data(self.message)                              # send instron 'start'
																				# connection : online / status : Idle
				self.uart.goal_status = 'Initializing'							# wait until status = Initializing
				self.uart.waitStatus()											# status : Initializing
			
				self.message['message'] = 'setting'
				self.uart.write_data(self.message)								# for update status
				
				GPIO.output(self.PIN, GPIO.HIGH)
				time.sleep(5)													# wait for gripper close

				self.uart.goal_status = 'Ready'									# wait until status = Ready
				self.uart.waitStatus()											# status : Ready


			if self.cmd_keys[i] == "execute":                         # experiment execute trigger

				self.uart.goal_status = 'Ready'									# check status : Ready
				self.uart.waitStatus()											# Status Ready

				self.message['message'] = 'experiment_start'
				self.uart.write_data(self.message)			                    # send instron 'experiment_start'
				self.uart.goal_status = 'Testing'								# wait until status = Testing
				self.uart.waitStatus()											# status : Testing

				self.message['message'] = 'running'
				self.uart.write_data(self.message)                              # for update status

				self.uart.goal_status = 'Done'									# wait until status = Done 
				self.uart.waitStatus()											# status : Done
				
				GPIO.output(self.PIN, False)
				time.sleep(5)													# wait for gripper open
				
				self.message['message'] = 'finish'
				self.uart.write_data(self.message)                              # tell Instron 'gripper closed'
				
				self.uart.goal_status = 'Idle'									# wait until status = Idle
				self.uart.waitStatus()											# status : Idle

		

if __name__=='__main__':

	jetson = Jetson()

	while not rospy.is_shutdown():
		pass



