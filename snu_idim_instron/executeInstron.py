#!/usr/bin/env python

import time
import serial
import sys
from autoRun import idim_smart_lab

class UART:

	def __init__(self):
		self.flag = 0
		self.state = 0
		self.autoRun = idim_smart_lab("testing")

	def write_data(self):
		serial_port.write("ok\n".encode())

	def read_data(self):
		data = ""
		while True:
			try:
				if serial_port.inWaiting() > 0:
					data = serial_port.readline().decode('utf-8').split('\n')[0]
					print(data)
					if data == '1':
						self.autoRun.execute('testing.txt')
						self.write_data()

			except KeyboardInterrupt:
				print('keyboard interrupt')
				sys.exit()

			except serial.serialutil.SerialException:
				print('error')

				serial_port.close()
				time.sleep(5)
				serial_port.open()



if __name__=='__main__':
	print("UART TTL communication")
	print("START")

	serial_port = serial.Serial(
	port="COM4",
	baudrate=115200,
	bytesize=serial.EIGHTBITS,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	timeout=0.05
	)
	u = UART()
	u.read_data()


