#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
from time import sleep
import json
import zmq
from threading import Thread
import serial
import serial.tools.list_ports

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from autoRun import *
from ArduinoInterface import ArduinoInterface

class DeviceClass_LaserCutter:
    def __init__(self, device_name='laser_cutter', ip_=None, port_=9448, usb_port_=None):

        ## GUI-based automation for laser cutter
        self.autoRun = idimAutomation(namespace='laser_cutter')
        self.arduino = ArduinoInterface(port=self.getSerialPort(desc='Arduino'))

        # Socket to talk to server
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("tcp://*:{}".format(port_))

        self.status = dict()
        self.status['device_type'] = 'LaserCutter'
        self.status['device_name'] = device_name
        self.status['connection'] = ''
        self.status['subject_name'] = ''
        self.status['status'] = ''
        self.status['recent_work'] = ''


        self.thread_1 = Thread(target=self.updateStatus)
        self.thread_1.start()

        self.thread_2 = Thread(target=self.zmqServer)
        self.thread_2.start()

        self.command({'connection': True})


    def __del__(self):
        ## Specialized del for the device (in this case, 3D printer)
        self.driver.close()
        self.thread_1.terminate()
        self.thread_2.terminate()

    def getSerialPort(self, desc='Arduino'):
        ports = list(serial.tools.list_ports.comports())
        port = ''
        for p in ports:
            if p.description.find(desc) != -1:
                port = p.device
        port = port if port != '' else None
        return port

    def zmqServer(self):
        while True:
            command = json.loads(self.socket.recv())
            print(command)
            self.command(command)
            self.socket.send_string(json.dumps(self.status))
            print(type(command), command)

    def updateStatus(self):
        while True:
            self.status = self.status  ## TODO


    
    def printStatus(self, status):
            print("\n==============================================================")
            print("[DEBUG] Device type: {}".format(self.status['device_type']))
            print("[DEBUG] Device name: {}".format(self.status['device_name']))
            print("[DEBUG] Connection: {}".format(self.status['connection']))
            print("[DEBUG] Status: {}".format(self.status['status']))
            print("[DEBUG] Subject name: {}".format(self.status['subject_name']))
            print("[DEBUG] Recent work: {}".format(self.status['recent_work']))
            
            print("\n[INFO] 3D Printer Status (#{})".format(self.device_id))
            print("\n  * Device:")
            print("    - Status: {}".format(status['connection']))
            print("    - File: {}".format(status['percentage']))
            print("    - Send ratio: {}".format(status['gcode_file']))
            print("    - Total time: {}".format(status['time_total']))
            print("    - Time elapsed: {}".format(status['time_elapsed']))
            print("    - Time left: {}".format(status['time_left']))

            print("\n  * Temperature:")
            print("    - Nozzle: {}".format(status['nozzle_temperature']))
            print("    - Bed: {}".format(status['bed_temperature']))


    def command(self, cmd_dict):
        print(type(cmd_dict), cmd_dict)
        cmd_keys = list(cmd_dict.keys())
        cmd_values = list(cmd_dict.values())
        print(len(cmd_keys))
        print(type(cmd_keys), cmd_keys)

        for i in range(len(cmd_keys)):
            if cmd_keys[i] == 'focus':
                self.status = 'focusing'
                print("Engrave!!!")
                self.autoRun.execute('{}.txt'.format('focus'))
            if cmd_keys[i] == 'engrave':
                self.status = 'engraving'
                print("Engrave!!!")
                self.autoRun.execute('{}.txt'.format('engrave'))
        self.status = 'Idle'




if __name__ == '__main__':  

    printer = DeviceClass_LaserCutter(device_name='laser_cutter', ip_='localhost', port_=5001, usb_port_=0)
    
    # print("[DEBUG] 1. Print start")
    # printer.command({'focus': 'demo_0'})
    # printer.command({'engrave': 'demo_0'})

