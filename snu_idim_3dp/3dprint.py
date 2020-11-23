#!/usr/bin/env python
#sudo service network-manager restart, sb

import time
import serial
import rospy
from std_msgs.msg import Float64, String
from idimAutomation import idim_smart_lab

class Printer:
	def __init__(self):
		rospy.init_node('printer_node')
		self.phase = 0
		self.printer_state = [0, 0, 0, 0]
		self.anet_flag = None
		self.automation = idim_smart_lab('printer')

		self.printer_publisher = rospy.Publisher("printer", String, queue_size=1)

		rospy.Subscriber('pc_to_printer', String, self.callback)

	def wait_for_complete(self):
		rospy.sleep(0.01)

	def callback(self, msg):
		cmds = msg.data.split(',')
		rospy.loginfo(cmds)

		for i in range(len(cmds)):
			cmd = int(cmds[i])
			rospy.loginfo("[3DP-{}] command: {}".format(i+1, cmd))
			self.wait_for_complete()
			if cmd == 0:
				self.printer_state[i] = cmd  # state: idle (0)
			elif cmd == 1:
				self.automation.execute('3DP_{}_start.txt'.format(i+1))
				self.printer_state[i] = cmd  # state: running (1)
			elif cmd == 2:
				self.automation.execute('3DP_{}_stop.txt'.format(i+1))
				self.printer_state[i] = cmd  # state: stopped (2)
			else:
				self.printer_state[i] = -1   # state: error (-1)


	def statePub(self):
		self.printer_publisher.publish('{},{},{},{}'.format(self.printer_state[0],self.printer_state[1],self.printer_state[2],self.printer_state[3]))

if __name__=='__main__':
	p = Printer()

	while not rospy.is_shutdown():
		p.statePub()
		rospy.sleep(1.0)
		pass