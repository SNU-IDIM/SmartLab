#!/usr/bin/env python

import time
import serial
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import String




class UART:
	def __init__(self, port_="/dev/ttyTHS1", baudrate_=115200):
		self.serial = serial.Serial(port=port_,
									baudrate=baudrate_,
									bytesize=serial.EIGHTBITS,
									parity=serial.PARITY_NONE,
									stopbits=serial.STOPBITS_ONE)


	def write_data(self, data):
		self.serial.write(data.encode())


	def read_data(self):
		data = ""
		while True:
			print('[DEBUG] Experiment - Running')
			if self.serial.inWaiting() > 0:
				data = self.serial.readline().decode('utf-8').split('\n')[0]
				if data == 'ok':
					print('[DEBUG] Experiment - Finnished')
					break
		return 1



class Jetson:
	def __init__(self):
		self.PIN = 15

		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.PIN, GPIO.OUT, initial=GPIO.LOW)

		rospy.init_node('jetson_node')
		rospy.Subscriber('instron/command', String, self.cmd_instron)
		self.instron_status_pub = rospy.Publisher("instron/status", String, queue_size=1)

		self.uart = UART()

		self.instron_status = 0
		self.instron_cmd    = 0

		print('[DEBUG] Node initialized !!!')


	def __del__(self):
		GPIO.cleanup()
		print('[DEBUG] Node is terminated !!!')


	def pub_status(self, status):
		status = str(status)
		self.instron_status_pub.publish(status)


	def cmd_instron(self, msg):
		self.instron_cmd = int(msg.data)
		print('[DEBUG] Instron command: {}'.format(self.instron_cmd))
		
		if self.instron_cmd == 1:
			print('[DEBUG] Experiment - Initialization')
			self.instron_status = 1 # 'start'
			self.pub_status(self.instron_status)
			rospy.sleep(1.0)

			print('[DEBUG] Experiment - Gripper close')
			self.instron_status = 1 # 'gripper close'
			self.pub_status(self.instron_status)
			GPIO.output(self.PIN, GPIO.HIGH)	

			print('[DEBUG] Experiment - Setting')
			self.instron_status = 1 # 'setting'
			self.pub_status(self.instron_status)
			self.uart.write_data("0\n")
			self.uart.read_data()
			time.sleep(5.0)

			print('[DEBUG] Experiment - Setting done')
			self.instron_status = 0 # 'done'
			self.pub_status(self.instron_status)


		if self.instron_cmd == 2:
			print('[DEBUG] Experiment - Start')
			self.instron_status = 2 # 'start'
			self.pub_status(self.instron_status)
			rospy.sleep(1.0)

			print('[DEBUG] Experiment - Running')
			self.instron_status = 2 # 'running'
			self.pub_status(self.instron_status)
			self.uart.write_data("1\n")
			self.uart.read_data()

			print('[DEBUG] Experiment - Gripper open')
			GPIO.output(self.PIN, False)
			self.instron_status = 2 # 'gripper open'
			self.pub_status(self.instron_status)

			print('[DEBUG] Experiment - Finished')
			time.sleep(5)
			self.instron_status = 0 # 'done'
			self.pub_status(self.instron_status)

		if self.instron_cmd == 3:
			pass
		

if __name__=='__main__':

	jetson = Jetson()

	while not rospy.is_shutdown():
		pass



