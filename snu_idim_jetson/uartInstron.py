#!/usr/bin/env python

import time
import serial
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import String
import numpy as np
import timeit



class UART:
	def __init__(self, port_="/dev/ttyTHS1", baudrate_=115200):
		self.serial = serial.Serial(port=port_,
									baudrate=baudrate_,
									bytesize=serial.EIGHTBITS,
									parity=serial.PARITY_NONE,
									stopbits=serial.STOPBITS_ONE)
		# print('serial ready')
		dictionary = ['connection','progress']
		self.serial.flushInput()
		self.continue_flag = 1



	def write_data(self, data):
		self.serial.write(data.encode())
		self.serial.flush()

	def read_data(self):
		start_time = timeit.default_timer()
		# print('ready to read data')
		while True:
			# run_time = timeit.default_timer()
			# if run_time - start_time <30:

			if self.serial.inWaiting() > 0 and self.continue_flag == 1 :
				self.serial.flushInput()
				time.sleep(0.1)
				self.data = self.serial.readline().decode('utf-8',errors = 'replace')
				self.data = self.serial.readline().decode('utf-8',errors = 'replace').split('\n')[0]
				print(self.data)

				# print('[DEBUG] Serial - Read')
				self.continue_flag = -1

				break

			else:
				# print('Not receiving Serial \n')
				continue

			# else:
			# 	self.data = 'progress no_respond!!!!!!!!!!!!!!!'
			# 	break


		return 1



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

		self.instron_status = 0
		self.instron_cmd    = 0
		data = ""

		self.status = dict()
		self.command = dict()


		self.status['connection'] = 'offline'								# connection (online/offline)			
		self.status['progress'] = 'waiting'                                 # progress initial value 'waiting'
		self.command['experiment'] = ''                                     # command from server (setup/execute)
		self.uart.write_data("online\n")                                    # check online to Instron
		time.sleep(1)
		self.updateStatus()										#debuging
		# print('[DEBUG] Node initialized !!!')



	def __del__(self):
		GPIO.cleanup()
		print('[DEBUG] Node is terminated !!!')


	def pub_status(self, status):
		status = str(status)
		self.instron_status_pub.publish(status)

	def updateStatus(self):
		self.uart.read_data()                                               # get string data from Instron
		dic_name = str(self.uart.data.split(' ')[0])                                   # dic name (connection/progress/etc)
		dic_data = str(self.uart.data.split(' ')[1])                                   # dic data (setup/gripper_close/etc)

		try:
			if dic_name == 'connection':
				self.status['connection'] = dic_data                        # connection data (online/offline)
				# print('[DEBUG] connection - updated')

			elif dic_name == 'progress':
				self.status['progress'] = dic_data							# progress data (setup, gripper_close, setting, etc..)
				
				# print('[DEBUG] progress - updated')							


		except:
			print("[ERROR] Status data loaded failed !!!")
		print('[DEBUG] {}'.format(self.status))

		return self.status

	def changeflag(self,var1,var2):
		if(self.status[var1] == var2):
			self.uart.continue_flag = 1

	def cmd_instron(self, msg):
		self.uart.serial.flushInput()
		self.instron_cmd = str(msg.data)
		#self.command['experiment'] = json.loads(self.instron_cmd)           # get command from server as 'string'
		self.command['experiment'] = self.instron_cmd

		print('[DEBUG] Instron command: {}'.format(self.command['experiment']))
		
		if self.command['experiment'] == "setup":                           # experiment setup trigger
			# print('[DEBUG] Experiment - Initialization')
			self.changeflag('connection','online')
			# print(self.uart.continue_flag)
			self.uart.write_data("start\n")                                 # ask instron status 'start'
			# self.uart.read_data()                                           # get 'start' response from Instron
			self.updateStatus()										#debuging
			# print(self.uart.continue_flag)

			self.changeflag('progress','setup_start')
		
			# print('[DEBUG] Experiment - Gripper close')
			self.uart.write_data("gripper_close\n")                         # ask instron status 'gripper_close'
			self.uart.serial.flushInput()
			GPIO.output(self.PIN, GPIO.HIGH)
			# self.uart.read_data()                                           # get 'gripper_close' response from Instron
			self.updateStatus()										#debuging

			self.changeflag('progress','gripper_close')

			# print('[DEBUG] Experiment - Setting')
			self.uart.write_data("setting\n")
			# self.uart.read_data()
			self.uart.serial.flushInput()
			time.sleep(0.1)
			self.updateStatus()										#debuging
			time.sleep(5.0)

			self.changeflag('progress','setting')

			# print('[DEBUG] Experiment - Setting done')
			self.uart.write_data("ready\n")
			self.uart.serial.flushInput()
			# self.uart.reasssd_data()
			self.updateStatus()										#debuging





		if self.command['experiment'] == "execute":                         # experiment execute trigger
			# print('[DEBUG] Experiment - execute')
			self.changeflag('progress','ready')
			self.uart.write_data("experiment_start\n")                      # ask instron status 'experiment_start'
			# self.uart.read_data()                                           # get 'experiment_start' response from Instron
			self.uart.serial.flushInput()
			self.updateStatus()										#debuging

			self.changeflag('progress','experiment_start')

			# print('[DEBUG] Experiment - Running')
			self.uart.write_data("running\n")                               # ask instron status 'running'
			# self.uart.read_data()                                           # get 'running' response from Instron
			self.uart.serial.flushInput()
			self.updateStatus()										#debuging

			self.changeflag('progress','running')

			# print('[DEBUG] Experiment - Gripper open')
			self.uart.write_data("gripper_open\n")                          # ask instron status 'gripper_open'
			GPIO.output(self.PIN, False)
			# self.uart.read_data()                                           # get 'gripper_open' response from Instron
			self.uart.serial.flushInput()
			self.updateStatus()										#debuging

			self.changeflag('progress','gripper_open')

			# print('[DEBUG] Experiment - Finished')
			time.sleep(5)
			self.uart.write_data("done\n")                                  # ask instron status 'done'
			# self.uart.read_data()                                           # get 'doen' response from Instron
			self.uart.serial.flushInput()
			self.updateStatus()										#debuging


		if self.command['experiment'] == "status":
			print('connection : {}, progress : {}'.format(self.status['connection'],self.status['progress']))
		

if __name__=='__main__':

	jetson = Jetson()

	while not rospy.is_shutdown():
		pass



