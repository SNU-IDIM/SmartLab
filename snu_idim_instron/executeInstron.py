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

CONNECTION_ONLINE = 1


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
		self.status = 'connection offline\n'


	def write_data(self, msg):
		self.serial_port.write(msg.encode('utf-8'))


	def execute(self, scripts=''):
		data = ''

		while True:
			try:

				if self.serial_port.inWaiting() > 0:
					data = self.serial_port.readline().decode('utf-8').split('\n')[0]
					time.sleep(0.1)

					print('[DEBUG] received data: {}'.format(data))
					
					if data == 'online':						# connection online
						self.status = 'connection online\n'
						self.write_data(self.status)

					elif data == 'start':						# setup_start
						self.status = 'progress setup_start\n'

					elif data == 'gripper_close':				# gripper_close
						self.status = 'progress gripper_close\n'

					elif data == 'setting':						# setting
						self.autoRun.execute(scripts[0])
						self.status = 'progress setting\n'

					elif data == 'ready':						# ready
						self.status = 'progress ready\n'

					elif data == 'experiment_start':			# experiment_start
						self.status = 'progress experiment_start\n'

					elif data == 'running':						# running
						self.autoRun.execute(scripts[1])
						self.status = 'progress running\n'

					elif data == 'gripper_open':				# gripper_open
						self.status = 'progerss gripper_open\n'

					elif data == 'done':						# done
						self.status= 'progress done\n'

					print('[DEBUG] sent data: {}'.format(self.status))

				self.write_data(self.status)

			except KeyboardInterrupt:
				print('keyboard interrupt')
				self.status = 'connection offline'
				self.write_data(self.status)

				sys.exit()

			except serial.serialutil.SerialException:
				print('error')

				self.serial_port.close()
				time.sleep(5)
				self.serial_port.open()

			except:
				self.status = 'connection offline'
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



