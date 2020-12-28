#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
from time import sleep
from copy import deepcopy
import json
from threading import Thread
import rospy
from std_msgs.msg import String

from DevicePluginToROS import DevicePluginToROS

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_3dp")) )
from DeviceClass_3DP import DeviceClass_3DP


class DeviceManager():
    def __init__(self):
        self.device_dict = dict()
        self.printer_list_idle     = []
        self.printer_list_initializing = []
        self.printer_list_printing = []
        self.printer_list_finished = []  ## ***
        self.printer_list_robot_done = []

        self.specimen_ready_list = []  ## ***

        self.test_ready_list = []  ## ***
        
        thread_1 = Thread(target=self.manager3DP)
        thread_1.start()


    def __del__(self):
        pass


    def addDevice(self, device_name, device_class=None):
        self.device_dict[device_name] = DevicePluginToROS(device_name=device_name, device_class=device_class)
        print("[DEBUG] '{}' is added to DeviceManager".format(device_name))
    

    def manager3DP(self):
        printing_queue = ['test1', 'test2', 'test3', 'test1', 'test2', 'test3']
        while True:
            n_printer = 0
            id_list_idle = []
            id_list_initializing= []
            id_list_printing = []
            id_list_finished = []
            
            for i in range(len(self.device_dict)):
                n_printer += 1 if self.device_dict[self.device_dict.keys()[i]].getStatus()['device_type'] == '3D Printer' else n_printer

                if self.device_dict[self.device_dict.keys()[i]].getStatus()['device_type'] == '3D Printer':
                    printer_id = self.device_dict[self.device_dict.keys()[i]].getStatus()['device_name']
                    printer_status = self.device_dict[self.device_dict.keys()[i]].getStatus()['status']

                    if printer_status.find('Idle') != -1:
                        id_list_idle.append(printer_id)
                    elif printer_status.find('Initializing') != -1:
                        id_list_initializing.append(printer_id)
                    elif printer_status.find('Printing') != -1:
                        id_list_printing.append(printer_id)
                    elif printer_status.find('Done') != -1:
                        id_list_finished.append(printer_id)

            self.printer_list_idle = id_list_idle
            self.printer_list_initializing = id_list_initializing
            self.printer_list_printing = id_list_printing
            self.printer_list_finished = id_list_finished

            printer_list_robot_done = self.printer_list_robot_done
            for printer_id in printer_list_robot_done:
                print(printer_id)
                self.printer_list_finished.remove(printer_id)
                self.printer_list_idle.append(printer_id)
                self.printer_list_robot_done.remove(printer_id)
                self.device_dict[printer_id].sendCommand({'status': 'Idle'})


            print("[INFO - DeviceManager] # of 3D Printers: {}".format(n_printer))
            print("[INFO - DeviceManager] # of 3D Printers in Idle: {}".format(len(self.printer_list_idle)), self.printer_list_idle)
            print("[INFO - DeviceManager] # of 3D Printers in Initializing: {}".format(len(self.printer_list_initializing)), self.printer_list_initializing)
            print("[INFO - DeviceManager] # of 3D Printers in Printing: {}".format(len(self.printer_list_printing)), self.printer_list_printing)
            print("[INFO - DeviceManager] # of 3D Printers in Finished: {}".format(len(self.printer_list_finished)), self.printer_list_finished)
            print("[INFO - DeviceManager] # of 3D Printers in Robot Job Done: {}".format(len(self.printer_list_robot_done)), self.printer_list_robot_done)


            print("[INFO - DeviceManager] Printing queue: {}".format(printing_queue))
            for printer_in_idle in self.printer_list_idle:
                try:
                    print_next = printing_queue.pop(0)
                    self.device_dict[printer_in_idle].sendCommand({'print': print_next})
                except:
                    print("[INFO - DeviceManager] Printing queue: empty !!!")
            


            sleep(3.0)


if __name__ == '__main__':

    rospy.init_node('DeviceManager')

    manager = DeviceManager()
    manager.addDevice('printer0', DeviceClass_3DP(device_name='printer0', ip_='192.168.60.101', port_='5001'))
    manager.addDevice('printer1', DeviceClass_3DP(device_name='printer1', ip_='192.168.60.101', port_='5002'))
    manager.addDevice('printer2', DeviceClass_3DP(device_name='printer2', ip_='192.168.60.101', port_='5003'))

    sleep(15.0)

    for i in range(3):
        manager.device_dict['printer{}'.format(i)].sendCommand({"connection": True})
    while True:
        sleep(3.0)
        if len(manager.printer_list_finished) != 0:
            print("[DEBUG] Robot Working................")
            sleep(10.0)
            manager.printer_list_robot_done.append(manager.printer_list_finished[0])
        # if manager.device_dict['printer0'].device_status['status'].find('Idle') != -1:
        #     hmanagerub.device_dict['printer0'].sendCommand({'print': '201122_feedrate_test'})