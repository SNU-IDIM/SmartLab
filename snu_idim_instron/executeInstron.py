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
import keyboard_recorder_save_Ver2



class autoInstron:

	def __init__(self, port='COM6', baud=115200, folder_dir='src'):
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
		self.result['E_modulus'] = 0
		self.result['U_stress'] = 0
		self.result['Raw_data'] = ''
		self.result['start_pic'] = ''
		self.result['mid_pic'] = ''
		self.result['finish_pic'] = ''
		self.result['Vision_data'] = ''
		self.result['plot'] = ''

		self.sql = mysql(user = 'IDIM-Instron', host = '192.168.60.21')
		self.record = keyboard_recorder_save_Ver2.Instron_cam()

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
			self.mid_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics/calibrate_img' + str(round(last_frame/2)) + '.png'
			self.finish_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics/calibrate_img' + str(last_frame) + '.png'
			self.plot_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/' + test_name + '_plot.png'
			self.E_Su_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/' + test_name + '_E_Su.txt'

		except:
			print("file is not exist")

	def read_data(self):
		data = []
		binary_image = [0,0,0,0,0,0]
		print("file raw")
		with open(self.raw_file, 'rb') as r1:
			data.append(r1.read())
			r1.close()
		print("file vision")
		with open(self.vision_file, 'rb') as r2:
			data.append(r2.read())
			r2.close()
		print("file start")
		with open(self.start_file, 'rb') as r3:
			data.append(r3.read())
			r3.close()
		print("file finish")
		with open(self.finish_file, 'rb') as r4:
			data.append(r4.read())
			r4.close()
		print("file mid")
		with open(self.mid_file, 'rb') as r5:
			data.append(r5.read())
			r5.close()
		print("file plot")
		with open(self.plot_file, 'rb') as r6:
			data.append(r6.read())
			r6.close()
		print("plot fin")
		for i in range(0,6):
			binary_image[i] = base64.b64encode(data[i])
			binary_image[i] = binary_image[i].decode('UTF-8')
		print("file E_Su")

		with open(self.E_Su_file, 'r') as r7:
			lines = r7.readline()
			E = lines.split(',')[0]
			ulti_stress = lines.split(',')[1]
			r7.close()

		print("E_Su fin")
		
		self.result['subject_name'] = self.subject_name
		self.result['Raw_data'] = binary_image[0]
		self.result['Vision_data'] = binary_image[1]
		self.result['start_pic'] = binary_image[2]
		self.result['mid_pic'] = binary_image[3]
		self.result['finish_pic'] = binary_image[4]
		self.result['plot'] = binary_image[5]
		self.result['E_modulus'] = str(E)
		self.result['U_stress'] = str(ulti_stress)
		print("result save")





	def execute(self, scripts=''):
		data = ''

		while True:
			try:
				self.serial_port.flushInput()
				# print("serial flushed")
				time.sleep(2)
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
						print("finish in")
						self.record.gripper(True)
						print("while before")
						while True:
							if self.record.stopsig() == False:
								# print(self.record.stopsig())
								pass
							elif self.record.stopsig() == True:
								print("stopsig",self.record.stopsig())
								break
						try:
							print("read file name")
							self.file_name(self.message['subject_name'])
							print("read data")
							self.read_data()
							print("send data")
							self.sql.sendResult(self.result)
						except:
							print("file doesn't exist")

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
						self.status['status'] = 'Serial_error1'
					
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

			except:
				self.status['status'] = 'Serial_error'
				print("except serial error")
				self.write_data(self.status)



if __name__=='__main__':
	print("[DEBUG] Instron Automation Started !!!")

	## Serial communication setting
	port = 'COM6'
	baud = 115200
	
	## Automation program setting
	folder_dir = 'src'
	scripts = ['start_experiment.txt', 'end_experiment.txt']

	## Create an instance (initialize)
	autoInstron = autoInstron(port=port, baud=baud, folder_dir=folder_dir)

	## Start automation
	autoInstron.execute(scripts)



