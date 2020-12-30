#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from autoRun import *

import time
import serial
import sys
import json



class autoInstron:

	def __init__(self, port='COM4', baud=115200, folder_dir='src'):
		self.serial_port = serial.Serial(port=port,
										 baudrate=baud,
										 bytesize=serial.EIGHTBITS,
										 parity=serial.PARITY_NONE,
										 stopbits=serial.STOPBITS_ONE,
										 timeout=0.05)
		self.flag = 0
		self.state = 0
		self.autoRun = idimAutomation(folder_dir)

		self.status = dict()
		self.status['connection'] = 'offline'
		self.status['status'] = 'waiting'
		self.message = dict()
		self.message['subject_name'] = 'NONE'
		self.message['message'] = ''
		self.subject_name  = 'NONE'
		self.result = []
		self.checklist = ['online','start','setting','experiment_start','running','finnish','send_data','data_saved']


	def write_data(self, msg):
		msg = json.dumps(msg)+str('\n')
		msg = msg.encode('utf-8')
		self.serial_port.write(msg)

	def data_send(self,name):
		print(name)
		with open ('C:\\Users\\IDIM-Instron\\Desktop\\Smart Laboratory\\' + str(name) + ".is_tens_RawData"+"\\Specimen_RawData_1.csv" ,"r") as res:
			print("1")
			self.raw_data = res.readlines()
			print(self.raw_data)
			self.status['result'] = self.raw_data

		
	# 		time.sleep(0.5)
	# 		self.write_data(self.raw_data)
	# 		time.sleep(0.5)
	# 		self.write_data(self.raw_data)
	# 		time.sleep(0.5)
	# 		self.write_data(self.raw_data)
	# 		time.sleep(0.5)
	# 		self.write_data(self.raw_data)
	# 		time.sleep(0.5)
	# 		self.write_data(self.raw_data)
	# 		time.sleep(0.5)
	# 		self.write_data(self.raw_data)
	# 		time.sleep(0.5)
	# 		self.write_data(self.raw_data)
	# 		time.sleep(0.5)
	# 		self.write_data(self.raw_data)
	# 		print(self.raw_data)

	def execute(self, scripts=''):
		data = ''

		while True:
			try:

				if self.serial_port.inWaiting() > 0:
					data = self.serial_port.readline().decode('utf-8').split('\n')[0]
					self.message = json.loads(data)

					time.sleep(0.1)

					print('[DEBUG] received data: {}'.format(self.message))
					
					if self.message['message'] == 'online':						# connection : online / status : Idle
						self.status['connection'] = 'online'
						self.status['status'] = 'Idle'


					elif self.message['message'] == 'start':						
						self.status['status'] = 'Initializing'	# status : Initializing
						self.autoRun.changeTXT(scripts[0], self.message['subject_name'])
						print(self.message['subject_name'])

					elif self.message['message'] == 'setting':						
						self.autoRun.execute(scripts[0])
						self.status['status'] = 'Ready'			# status : Ready
						self.autoRun.returnTXT(scripts[0], self.message['subject_name'])

					elif self.message['message'] == 'experiment_start':			
						self.status['status'] = 'Testing'			# status : Testing

					elif self.message['message'] == 'running':						
						self.autoRun.execute(scripts[1])
						self.status['status'] = 'Done'			# status : Done

					elif self.message['message'] == 'finish':						
						self.status['status']= 'Idle'			# status : Idle
						self.message['subject_name'] = 'NONE'

					elif self.message['message'] =='send_data':
						self.data_send(self.message['subject_name'])
						time.sleep(.5)
						self.status['status'] = 'data_sent'

					elif self.message['message'] == 'data_saved':
						if 'result' in self.status.keys():
							self.status['result'] = ''

							del(self.status['result'])
						self.status['status'] = 'Idle'
						

					elif self.message['message'] not in self.checklist:
						if 'result' in self.status.keys():
							self.status['result'] = ''
							del(self.status['result'])
						self.status['status'] = 'Serial_error'

						
					

					print('[DEBUG] sent data: {}'.format(self.status))

				self.write_data(self.status)

			except KeyboardInterrupt:
				self.status['connection'] = 'offline'
				self.write_data(self.status)

				sys.exit()

			except serial.serialutil.SerialException:
				print('error')

				self.serial_port.close()
				time.sleep(5)
				self.serial_port.open()

			except :
				self.status['status'] = 'Serial_error'
				self.write_data(self.status)
				self.write_data(self.status)
				self.write_data(self.status)



if __name__=='__main__':
	print("[DEBUG] Instron Automation Started !!!")

	## Serial communication setting
	port = 'COM5'
	baud = 115200
	
	## Automation program setting
	folder_dir = 'src'
	scripts = ['start_experiment.txt', 'end_experiment.txt']

	## Create an instance (initialize)
	autoInstron = autoInstron(port=port, baud=baud, folder_dir=folder_dir)

	## Start automation
	autoInstron.execute(scripts)



