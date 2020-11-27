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


	def write_data(self):
		self.serial_port.write("ok\n".encode())


	def execute(self, script='Instron'):
		data = ""
		while True:
			try:
				if self.serial_port.inWaiting() > 0:
					data = self.serial_port.readline().decode('utf-8').split('\n')[0]
					print(data)
					if data == '1':
						self.autoRun.execute('{}.txt'.format(script))
						self.write_data()

			except KeyboardInterrupt:
				print('keyboard interrupt')
				sys.exit()

			except serial.serialutil.SerialException:
				print('error')

				self.serial_port.close()
				time.sleep(5)
				self.serial_port.open()



if __name__=='__main__':
	print("[DEBUG] Instron Automation Started !!!")

	## Serial communication setting
	port = 'COM6'
	baud = 115200
	
	## Automation program setting
	folder_dir = 'src'
	script = 'Instron'

	## Create an instance (initialize)
	autoInstron = autoInstron(port=port, baud=baud, folder_dir=folder_dir)
	autoInstron.autoRun.execute('{}.txt'.format(script))

	## Start automation
	autoInstron.execute(script)


