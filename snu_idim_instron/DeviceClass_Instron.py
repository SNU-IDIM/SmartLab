#!/usr/bin/env python

import time
import serial
import Jetson.GPIO as GPIO
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
		self.status['subject_name'] = None
		self.status['connection'] = 'Offline'
		self.status['status'] = None

		self.message = dict()  # message from Jetson to Instron
		self.message['message'] = 'online'
		self.message['subject_name'] = ''
		
		self.newstatus = dict()

		self.thread_1 = threading.Thread(target=self.updateStatus)
		self.thread_1.daemon = True
		self.thread_1.start()

	def __del__(self):
		self.thread_1.terminate()


	def changeflag(self, var):
		# print("changeflage")
		# print(self.status['status'], var)

		if self.status['status'] == var:
			self.continue_flag = 1

		elif self.status['status'] == 'Serial_error':
			print('serialerror')
			self.continue_flag = 0
			
		elif self.status['status'] == 'Serial_error1':
			print('serialerror1')
			self.continue_flag = 0
			
		else:
			self.continue_flag = -1


	def write_data(self, data):
		data = json.dumps(data) + str('\n')
		self.serial.write(data.encode())

	def updateStatus(self):
		while True:

			if self.serial.inWaiting() > 0:
				self.serial.flushInput()
				time.sleep(0.7)

				self.data = self.serial.readline().split('\n')#.decode('utf-8',errors = 'replace')
				self.data = self.serial.readline().decode('utf-8', errors='replace').split('\n')[0]
				self.newstatus = json.loads(self.data)
				self.status.update(self.newstatus)
			else:
				# print('Waiting for Serial \n')
				continue


	def waitStatus(self):
		self.count = 0
		ntime = time.time()
		while True:
			self.changeflag(self.goal_status)
			if self.continue_flag == 1:
				self.continue_flag = -1
				print('[Status] {}'.format(self.status))
				break

			elif self.continue_flag == 0:
				if self.message['message'] == 'finish':
					print(self.message, self.count)
					self.count += 1
					# time.sleep(1)

				# if self.count <4:
					# print(self.message, self.count)
					self.write_data(self.message)
					print("write data again")
					time.sleep(0.3)
				elif time.time() - ntime > 2:
					print("Waiting for the next step\n")
					self.write_data(self.message)
					ntime = time.time()

	def datacollect(self):
		print("data collect")
		self.prev_result = 'empty'
		self.result = []
		self.linecount = 0

		while True:
			if self.status['status'] == 'data_sent':
				break

			if self.status['result'] != self.prev_result:
				self.prev_result = self.status['result']
				self.result.append(json.loads(self.prev_result))
				self.linecount = self.linecount + 1
				self.message['line'] = self.linecount
				print(self.linecount)

			print(self.message)
			self.write_data(self.message)
			time.sleep(1)


class DeviceClass_Instron:

	def __init__(self, device_name='instron'):
		## Common init for all devices
		self.status = dict()
		self.status['device_type'] = 'Universal Testing Machine'
		self.status['device_name'] = device_name
		self.status['connection'] = ''
		self.status['subject_name'] = ''
		self.status['status'] = ''
		self.status['recent_work'] = ''

		# # Jetson nano board setting for digital I/O
		self.PIN = 15
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.PIN, GPIO.OUT, initial=GPIO.LOW)



		## UART communication with Instron PC
		self.uart = UART()
		self.uart.write_data(self.uart.message)  # Connection check (Instron PC)
		time.sleep(1)
		self.uart.goal_status = 'Idle'  # wait until status = 'Idle'
		# print(self.uart.status)
		self.uart.waitStatus()  # status : 'Idle'

		self.thread_1 = threading.Thread(target=self.updateStatus)
		self.thread_1.start()

	
	def __del__(self):
		GPIO.cleanup()
		self.thread_1.terminate()
		print('[DEBUG] Node is terminated !!!')

	
	def updateStatus(self):
		while True:
			self.status.update(self.uart.status)
			time.sleep(0.1)


	def command(self, cmd_dict):

		cmd_keys = cmd_dict.keys()
		cmd_values = cmd_dict.values()

		print('[DEBUG] Instron command: {}'.format(cmd_dict))
		for i in range(len(cmd_keys)):
			# if cmd_keys[i] == "serial_check":
			# 	self.uart.message['message'] = 'online'
			# 	self.uart.write_data(self.uart.message)			                # send instron 'serial_check'
			# 	self.uart.goal_status = 'connected'
			# 	print("waiting")
			# 	self.uart.waitStatus()
			# 	print("serial_connected")
			# 	self.uart.status['status'] = 'Idle'

			if cmd_keys[i] == "setup":                         			# experiment setup trigger
				self.uart.status['subject_name'] = cmd_values[i]
				self.uart.message['subject_name'] = cmd_values[i]
				self.uart.message['message'] = 'start'
				self.uart.write_data(self.uart.message)                         # send instron 'start'
				print("setupmessage :", self.uart.message)
																				# connection : online / status : Idle
				self.uart.goal_status = 'Initializing'							# wait until status = Initializing
				print("initializing goal set tup")
				self.uart.waitStatus()											# status : Initializing
				print("initializing waiting")
				self.uart.message['message'] = 'setting'
				self.uart.write_data(self.uart.message)								# for update status
				
				GPIO.output(self.PIN, GPIO.HIGH)
				time.sleep(5)													# wait for gripper close

				self.uart.goal_status = 'Ready'									# wait until status = Ready
				self.uart.waitStatus()											# status : Ready

			if cmd_keys[i] == "execute":                         # experiment execute trigger
				print('execute in')
				self.uart.goal_status = 'Ready'									# check status : Ready
				self.uart.waitStatus()											# Status Ready

				self.uart.message['message'] = 'experiment_start'
				self.uart.write_data(self.uart.message)			                # send instron 'experiment_start'
				self.uart.goal_status = 'Testing'								# wait until status = Testing
				print("waitstatus testing")
				self.uart.waitStatus()											# status : Testing

				self.uart.message['message'] = 'running'
				self.uart.write_data(self.uart.message)                         # for update status

				self.uart.goal_status = 'Done'									# wait until status = Done 
				self.uart.waitStatus()											# status : Done

			if cmd_keys[i] == "open":
				GPIO.output(self.PIN, GPIO.LOW)
				# time.sleep(0.5)												# wait for gripper open

				self.uart.message['message'] = 'finish'
				now = time.time()
				while (time.time() - now)< 2:
					print(time.time())
					self.uart.write_data(self.uart.message)                     # tell Instron 'gripper closed'
					time.sleep(0.1)
				self.uart.goal_status = 'Idle'									# wait until status = Idle
				self.uart.waitStatus()											# status : Idle
		

		

							

		

if __name__=='__main__':

	instron = DeviceClass_Instron()

	while True:
		time.sleep(1.0)
		pass