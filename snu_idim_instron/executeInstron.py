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

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_strain")) )
import keyboard_recorder_save_Ver



class autoInstron:

	def __init__(self, port='COM8', baud=115200, folder_dir='src'):
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
		self.status['connection'] = 'Offline'
		self.status['status'] = None
		self.message = dict()
		self.message['subject_name'] = None
		self.message['message'] = ''
		self.subject_name  = None
		self.checklist = ['online','start','setting','experiment_start','running','finish','send_data','data_saved']

		self.record = keyboard_recorder_save_Ver.Instron_cam()
		

	def write_data(self, msg):
		# print("writeerror")

		# print(msg)
		msg = json.dumps(msg)+str('\n')
		# print(msg)
		msg = msg.encode('utf-8')
		self.serial_port.write(msg)
		# print("writeerrorfinish")


	def data_send(self,name,linenum):
		with open ('C:\\Users\\IDIM-Instron\\Desktop\\Smart Laboratory\\' + str(name) + ".is_tens_RawData"+"\\Specimen_RawData_1.csv" ,"r") as res:
			self.raw_data = res.readlines()
			self.line = len(self.raw_data)
			if linenum < self.line:
				self.raw_data = self.raw_data[linenum]
				print(self.raw_data)
				self.json_raw_data = json.dumps(self.raw_data)
				self.status['result'] = self.json_raw_data
			else:
				self.status['status'] = 'data_sent'


	def execute(self, scripts=''):
		data = ''

		while True:
			try:
				self.serial_port.flushInput()
				time.sleep(0.5)

				if self.serial_port.inWaiting() > 0:

					data = self.serial_port.readline().decode('utf-8').split('\n')[0]
					print("debug: {}".format(data))
					self.message = json.loads(data)

					time.sleep(0.1)

					print('[DEBUG] received data: {}'.format(self.message))
					
					if self.message['message'] == 'online':						# connection : online / status : Idle
						self.status['connection'] = 'Online'
						self.status['status'] = 'Idle'


					elif self.message['message'] == 'start':						
						self.status['status'] = 'Initializing'	# status : Initializing
						self.subject_name = self.message['subject_name']
						self.autoRun.changeTXT(scripts[0], self.subject_name)
						print(self.message['subject_name'])

					elif self.message['message'] == 'setting':						
						self.autoRun.execute(scripts[0])
						self.status['status'] = 'Ready'			# status : Ready
						self.autoRun.returnTXT(scripts[0], self.subject_name)

					elif self.message['message'] == 'experiment_start':			
						self.status['status'] = 'Testing'			# status : Testing

					elif self.message['message'] == 'running':						
						
						self.record.trigger('recordstart',self.subject_name)
						self.autoRun.execute(scripts[1])
						self.record.trigger('recordstop',self.subject_name)
						self.status['status'] = 'Done'			# status : Done

					elif self.message['message'] == 'finish':	 					
						self.status['status']= 'Idle'			# status : Idle
						self.message['subject_name'] = 'NONE'

					elif self.message['message'] =='send_data':
						self.status['status'] = 'sending_data'
						self.data_send(self.message['subject_name'],self.message['line'])
						# time.sleep(.5)
						# self.status['status'] = 'data_sent'

					elif self.message['message'] == 'data_saved':
						if 'result' in self.status.keys():
							self.status['subject_name'] = 'NONE'
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
				self.status['connection'] = 'Offline'
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



if __name__=='__main__':
	print("[DEBUG] Instron Automation Started !!!")

	## Serial communication setting
	port = 'COM8'
	baud = 115200
	
	## Automation program setting
	folder_dir = 'src'
	scripts = ['start_experiment.txt', 'end_experiment.txt']

	## Create an instance (initialize)
	autoInstron = autoInstron(port=port, baud=baud, folder_dir=folder_dir)

	## Start automation
	autoInstron.execute(scripts)



