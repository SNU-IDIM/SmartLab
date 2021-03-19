#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from autoRun import *
from datasql import mysql

import time
import serial
import sys
import json
import pandas as pd
from PIL import Image
import base64
from io import BytesIO

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
		self.checklist = ['online','serial_check','start','setting','experiment_start','running','finish']
		self.result = dict()
		self.result['subject_name'] = 'None'
		self.result['Raw_data'] = ''
		self.result['start_pic'] = ''
		self.result['finish_pic'] = ''
		self.result['Vision_data'] = ''

		self.record = keyboard_recorder_save_Ver.Instron_cam()
		self.sql = mysql(user = 'IDIM-Instron', host = '192.168.60.21')

	def write_data(self, msg):
		# print("writeerror")

		# print(msg)
		msg = json.dumps(msg)+str('\n')
		# print(msg)
		msg = msg.encode('utf-8')
		self.serial_port.write(msg)
		# print("writeerrorfinish")


	def file_name(self,test_name):
		last_frame = len(os.listdir('C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics')) -1
		try:
			self.raw_file = 'C:/Users/IDIM-Instron/Desktop/Smart Laboratory/' + test_name + '.is_tens_RawData/Specimen_RawData_1.csv'
			self.vision_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/' + test_name + '__vision___.xlsx'
			self.start_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics/calibrate_img0.png'
			self.finish_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics/calibrate_img' + str(last_frame) + '.png'

		except:
			print("file is not exist")

	def read_data(self):
		data = []
		binary_image = [0,0,0,0]
		with open(self.raw_file, 'rb') as r1:
			data.append(r1.read())
			r1.close()
		with open(self.vision_file, 'rb') as r2:
			data.append(r2.read())
			r2.close()
		with open(self.start_file, 'rb') as r3:
			data.append(r3.read())
			r3.close()
		with open(self.finish_file, 'rb') as r4:
			data.append(r4.read())
			r4.close()

		for i in range(0,4):
			binary_image[i] = base64.b64encode(data[i])
			binary_image[i] = binary_image[i].decode('UTF-8')

		
		self.result['subject_name'] = self.subject_name
		self.result['Raw_data'] = binary_image[0]
		self.result['Vision_data'] = binary_image[1]
		self.result['start_pic'] = binary_image[2]
		self.result['finish_pic'] = binary_image[3]
		# self.result['plot'] = binary_image[4]





	def execute(self, scripts=''):
		data = ''

		while True:
			try:
				self.serial_port.flushInput()
				time.sleep(1)
				if self.serial_port.inWaiting() > 0:

					data = self.serial_port.readline().decode('utf-8').split('\n')[0]
					print("debug: {}".format(data))
					self.message = json.loads(data)

					time.sleep(0.1)

					print('[DEBUG] received data: {}'.format(self.message))
					
					if self.message['message'] == 'online':						# connection : online / status : Idle
						self.status['connection'] = 'Online'
						self.status['status'] = 'Idle'

					# elif self.message['message'] == 'serial_check':
					# 	time.sleep(1)
					# 	self.status['status'] = 'Connected'


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
						while True:
							if self.record.stopsig() == False:
								print(self.record.stopsig())
								pass
							elif self.record.stopsig() == True:
								print(self.record.stopsig())
								break
						self.file_name(self.message['subject_name'])
						self.read_data()
						self.sql.sendResult(self.result)

						self.status['status'] = 'Idle'


					# elif self.message['message'] == 'data_saved':
					# 	if 'result' in self.status.keys():
					# 		self.status['subject_name'] = 'NONE'
					# 		del(self.status['result'])
					# 	self.status['status'] = 'Idle'
						

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
	port = 'COM13'
	baud = 115200
	
	## Automation program setting
	folder_dir = 'src'
	scripts = ['start_experiment.txt', 'end_experiment.txt']

	## Create an instance (initialize)
	autoInstron = autoInstron(port=port, baud=baud, folder_dir=folder_dir)

	## Start automation
	autoInstron.execute(scripts)



