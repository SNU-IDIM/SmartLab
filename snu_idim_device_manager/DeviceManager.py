#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
from time import sleep
import json
from threading import Thread
import rospy
from std_msgs.msg import String

from DevicePluginToROS import DevicePluginToROS

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_3dp")) )
from DeviceClass_3DP import DeviceClass_3DP


class DeviceHUB():
    def __init__(self):
        self.device_dict = dict()
        
        thread_1 = Thread(target=self.manager3DP)
        thread_1.start()


    def __del__(self):
        pass


    def addDevice(self, device_name):
        self.device_dict[device_name] = DevicePluginToROS(device_name=device_name, device_class=None)
        print("[DEBUG] '{}' is added to DeviceHUB".format(device_name))
    

    def manager3DP(self):
        while True:
            n_printer = 0
            id_list_idle = []
            id_list_printing = []
            id_list_finished = []
            
            for i in range(len(self.device_dict)):
                n_printer += 1 if self.device_dict[self.device_dict.keys()[i]].getStatus()['device_type'] == '3D Printer' else n_printer

                if self.device_dict[self.device_dict.keys()[i]].getStatus()['device_type'] == '3D Printer':
                    printer_id = self.device_dict[self.device_dict.keys()[i]].getStatus()['device_name']
                    printer_status = self.device_dict[self.device_dict.keys()[i]].getStatus()['status']
                    if printer_status.find('Idle') != -1 or printer_status.find('Done') != -1:
                        id_list_idle.append(printer_id)
                        
            print("[INFO - DeviceHUB] # of 3D Printers: {}".format(n_printer))
            print("[INFO - DeviceHUB] # of 3D Printers in Idle: {}".format(id_list_idle))
            try:
                print(self.device_dict[id_list_idle[0]])
            except:
                print("ERROR")
                pass



            sleep(3.0)

        



# def printStatus3DP(device_list):
#     for device in device_list:
#         status = device.getStatus()
#         try:
#             print("\n  * Device: {}".format(device.device_name))
#             print("    - Status: {}".format(status['connection']))
#             print("    - File: {}".format(status['percentage']))
#             print("    - Send ratio: {}".format(status['gcode_name']))
#             print("    - Total time: {}".format(status['time_total']))
#             print("    - Time elapsed: {}".format(status['time_elapsed']))
#             print("    - Time left: {}".format(status['time_left']))

#             print("\n  * Temperature:")
#             print("    - Nozzle: {}".format(status['nozzle_temperature']))
#             print("    - Bed: {}".format(status['bed_temperature']))
#         except:
#             print("[ERROR]")


# def printingManager(device_list):
#     n_printer = 0
#     printer_idx = []
#     for i in range(len(device_list)):
#         n_printer += 1 if device_list[i].getStatus()['device_type'] == '3D Printer' else n_printer
#     print("[DEBUG] # of 3D Printers: {}".format(n_printer))


# def addDevice(device_dict, device_name):
#     device_dict[device_name] = DevicePluginToROS(device_name=device_name, device_class=None)


if __name__ == '__main__':

    rospy.init_node('DeviceHUB')

    hub = DeviceHUB()
    hub.addDevice('printer0')

    # print_queue = []
    # device_list = []
    # device_dict = dict()

    # addDevice(device_dict, 'printer0')
    # # addDevice(device_dict, 'printer1')
    # # addDevice(device_dict, 'printer2')
    # # addDevice(device_dict, 'printer3')
    # rospy.sleep(5.0)
    


    # print("[INFO - DeviceHUB] # of connected device: {}".format(len(device_dict)))
    # print("[INFO - DeviceHUB] Device list: {}".format(device_dict.keys()))

    # n_printer = 0
    # printer_idx = []
    # for i in range(len(device_dict)):
    #     n_printer += 1 if device_dict[device_dict.keys()[i]].getStatus()['device_type'] == '3D Printer' else n_printer
    # print("[INFO - DeviceHUB] # of 3D Printers: {}".format(n_printer))

    
    # # device_dict['printer0'].sendCommand({'connection': True})


    # rospy.sleep(5.0)
    # device_dict['printer0'].sendCommand({'print': '201122_feedrate_test'})


    # while True:
    #     # device_list[0].sendCommand({'connection': True})
    #     printStatus3DP(device_list)
    #     rospy.sleep(2.0)


    # # mode = 'manager'
    # # # mode = 'device'

    # # if mode == 'device':
    # #     sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../../snu_idim_3dp")) )
    # #     from Automate3DP import Automate3DP

    # #     device_name = 'printer0'
    # #     device_class = Automate3DP(device_name)
    # #     hub_device_part = DeviceHUB(device_name=device_name, device_class=Automate3DP(device_name))
    
    # # elif mode == 'manager':
    # #     device_name = 'printer0'
    # #     device_class = None

    # #     rospy.init_node('this_node_is_not_necessary_for_real_implementation')
    # #     hub_manager_part = DeviceHUB(device_name=device_name, device_class=None);   sleep(3)

    # #     cmd_dict = {'connection': True}
    # #     hub_manager_part.manager_sendCommand(cmd_dict);                             sleep(10)

    # #     status = hub_manager_part.manager_getStatus()
    # #     print("[INFO] Current 'status_dict': \n{}".format(status))