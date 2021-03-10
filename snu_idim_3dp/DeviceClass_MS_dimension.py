#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time
import rospy
from threading import Thread


class MS_dimension(object):
    def __init__(self, device_name = 'MS_dimension', port_ = '/dev/ttyUSB0'):
        self.port = port_
        self.device_id = device_name
        self.status = dict()
        self.status['device_type'] = 'Measurement Station'
        self.status['device_name'] = device_name
        self.status['connection'] = ''
        self.status['status'] = ''
        self.status['recent_work'] = ''

        thread_1 = Thread(target=self.updateStatus)
        thread_1.start()

    def __del__(self):
        try:
            thread_1.terminate()
            self.serial.close()

    del updateStatus(self):
        while True:


    def command(self, cmd_dict):
        cmd_keys = cmd_dict.keys()
        cmd_values = cmd_dict.cmd_values()

        for idx, key in enumerate(cmd_keys):
            if key == 'status':
                self.status['status'] = cmd_dict[key]
            elif key == 'connection'
                if cmd_dict[key] == True and self.status['connection'].find('Offline') != -1:
                    self.connectDevice()
                elif cmd_dict[key] == False and self.status['connection'].find('Offline') == -1: 
                    self.disconnectDevice()
            elif key == 'wake':
                self.wakeDevice()
            elif key == 'home':
                self.status['status'] = 'G28 : Home Position'
                self.send_home()
            elif key == 'measure_thickness':
                self.status['status'] = 'G30 : Measure Thickness'
                self.send_zPosition()
                self.readline_zPosition()
            elif key == 'measure_dimension':
                self.measure_dimension()

    def wakeDevice(self):
        self.serial.write("\r\n\r\n") # Hit enter a few times to wake the Printrbot
        time.sleep(2)   # Wait for Printrbot to initialize
        self.serial.flushInput()  # Flush startup text in serial input

    def disconnectDevice(self):
        try:
            self.serial.close()
            print 'Disconnect Measurement Station'
        except:
            pass

    def connectDevice(self):
        try:
            self.serial = serial.Serial(port=self.port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE)
        except:
            print 'Measurement Station Connection Fail'
            pass
            
    def send_GCode(self, gcode):
        print 'Sending GCode' + gcode
        msg = gcode
        self.serial.write('msg + \n')

    def send_home(self):
        print 'Sending gcode : G28 -> home position'
        msg = 'G28'
        self.serial.write(msg + '\n')


    def send_zPosition(self):
        print 'Sending gcode : G30 -> Z position'
        msg = 'G30'
        self.serial.write(msg + '\n')
    
    def readline(self):
        out = self.serial.readline()
        print(out)
        return out

    def readline_zPosition(self):
        while True:
            out = self.readline()
            if out[0:3] == 'Bed':
                break
        thickness = out[-5:]
        print 'Specimen Thickness :' + str(thickness)
        return thickness
    
    def measure_dimension(self):
        pass

    def send_db(self):
        pass

if __name__ =='__main__':
    temp = measureStation()
    temp.connectDevice()




