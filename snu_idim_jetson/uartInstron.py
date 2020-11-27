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
		self.serial.write(data)


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
		self.phase = 0
		self.instron_flag = None
		self.uart = UART()

		self.jetson_publisher = rospy.Publisher("jetson", String, queue_size=1)

		rospy.Subscriber('pc_to_jetson', Float64, self.callback)


	def wait_for_complete(self):
		rospy.sleep(0.1)
	

	def callback(self, msg):
		rospy.loginfo(msg.data)
		self.start_flag = msg.data
		
		if (self.start_flag == 10):
			print('[DEBUG] Experiment - Start')
			self.jetson_publisher.publish("start")
			self.wait_for_complete()

			print('[DEBUG] Experiment - Gripper close')
			self.jetson_publisher.publish("gripper close")
			GPIO.output(IN1, GPIO.HIGH)	
			# time.sleep(10)

			print('[DEBUG] Experiment - Running')
			self.jetson_publisher.publish("running")
			self.uart.write_data("1\n")
			self.uart.read_data()

			print('[DEBUG] Experiment - Finnished')
			self.jetson_publisher.publish("done")
			# GPIO.output(IN1, False)
			time.sleep(5)


		if (self.start_flag == 20):
			rospy.loginfo("Start_flag = %s",self.start_flag)
			#do something ex) self.open_valve
			#self.jetson_publisher.publish("running")
			self.wait_for_complete()
			self.instron_flag = 1
			# GPIO.output(IN1, False)
			time.sleep(5)

		if (self.start_flag == 30):
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


