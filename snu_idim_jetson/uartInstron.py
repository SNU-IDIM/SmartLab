#!/usr/bin/env python

import time
import serial
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Float64, String

IN1 = 15



class UART:
	def __init__(self, port_="/dev/ttyTHS1", baudrate_=115200):
		self.serial = serial.Serial(
									port=port_,
									baudrate=baudrate_,
									bytesize=serial.EIGHTBITS,
									parity=serial.PARITY_NONE,
									stopbits=serial.STOPBITS_ONE,
									)


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



class JET:
	def __init__(self):
		rospy.init_node('jetson_node')
		rospy.Subscriber('instron/command', Float64, self.cmd_instron)
		self.instron_status_pub = rospy.Publisher("instron/status", String, queue_size=1)

		self.uart = UART()

		self.instron_status = 0
		self.instron_cmd    = 0

		print('[DEBUG] Node initialized !!!')


	def pub_status(self, status):
		status = str(status)
		self.instron_status_pub.publish(status)


	def wait_for_complete(self):
		rospy.sleep(0.1)
	

	def cmd_instron(self, msg):
		self.instron_cmd = int(msg.data)
		print('[DEBUG] Instron command: {}'.format(cmd))
		
		if (self.instron_cmd == 1):
			print('[DEBUG] Experiment - Initialization')
			self.instron_status = 1 # 'start'
			self.pub_status(self.instron_status)
			self.wait_for_complete()

			print('[DEBUG] Experiment - Gripper close')
			self.instron_status = 1 # 'gripper close'
			self.pub_status(self.instron_status)
			GPIO.output(IN1, GPIO.HIGH)	

			print('[DEBUG] Experiment - Setting')
			self.instron_status = 1 # 'setting'
			self.pub_status(self.instron_status)
			self.uart.write_data("0\n")
			self.uart.read_data()
			time.sleep(5.0)

			print('[DEBUG] Experiment - Setting done')
			self.instron_status = 0 # 'done'
			self.pub_status(self.instron_status)


		if (self.instron_cmd == 2):
			print('[DEBUG] Experiment - Start')
			self.instron_status = 2 # 'start'
			self.pub_status(self.instron_status)
			self.wait_for_complete()

			print('[DEBUG] Experiment - Running')
			self.instron_status = 2 # 'running'
			self.pub_status(self.instron_status)
			self.uart.write_data("1\n")
			self.uart.read_data()

			print('[DEBUG] Experiment - Gripper open')
			GPIO.output(IN1, False)
			self.instron_status = 2 # 'gripper open'
			self.pub_status(self.instron_status)

			print('[DEBUG] Experiment - Finished')
			time.sleep(5)
			self.instron_status = 0 # 'done'
			self.pub_status(self.instron_status)

		if (cmd == 3):
			rospy.loginfo("Start_flag = %s",self.start_flag)
			#do something ex) self.open_valve
			#self.jetson_publisher.publish("finish")
			self.wait_for_complete()
			self.instron_flag = 0
			print("FINISH")
		

if __name__=='__main__':

	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)


	j = JET()


	while not rospy.is_shutdown():
		pass


	GPIO.cleanup()


