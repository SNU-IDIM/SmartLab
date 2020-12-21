#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
from time import sleep
import json

import rospy
from std_msgs.msg import String

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from DevicePluginToROS import DevicePluginToROS

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_3dp")) )
from Automate3DP import Automate3DP

def printStatus3DP(device_list):
    for device in device_list:
        status = device.getStatus()
        try:
            print("\n  * Device: {}".format(device.device_name))
            print("    - Status: {}".format(status['connection']))
            print("    - File: {}".format(status['percentage']))
            print("    - Send ratio: {}".format(status['gcode_name']))
            print("    - Total time: {}".format(status['time_total']))
            print("    - Time elapsed: {}".format(status['time_elapsed']))
            print("    - Time left: {}".format(status['time_left']))

            print("\n  * Temperature:")
            print("    - Nozzle: {}".format(status['nozzle_temperature']))
            print("    - Bed: {}".format(status['bed_temperature']))
        except:
            print("[ERROR]")



if __name__ == '__main__':

    rospy.init_node('DeviceHUB')

    device_list = []

    device0 = DevicePluginToROS(device_name='printer0', device_class=None);    device_list.append(device0)
    device1 = DevicePluginToROS(device_name='printer1', device_class=None);    device_list.append(device1)
    device2 = DevicePluginToROS(device_name='printer2', device_class=None);    device_list.append(device2)
    # device_3 = DevicePluginToROS(device_name='printer3', device_class=None);    device_list.append(device_3)

    

    device_name_list = []
    for device in device_list:
        device_name_list.append(device.device_name)

    print("[INFO - DeviceHUB] # of connected device: {}".format(len(device_list)))
    print("[INFO - DeviceHUB] Device list: {}".format(device_name_list))
    
    rospy.sleep(5.0)
    device0.sendCommand({'connection': True})
    device1.sendCommand({'connection': True})
    device2.sendCommand({'connection': True})

    rospy.sleep(5.0)
    device0.sendCommand({'print': 'connection_test.gcode'})
    device1.sendCommand({'print': 'connection_test.gcode'})
    device2.sendCommand({'print': 'connection_test.gcode'})

    while True:
        # device_list[0].sendCommand({'connection': True})
        printStatus3DP(device_list)
        rospy.sleep(2.0)


    # mode = 'manager'
    # # mode = 'device'

    # if mode == 'device':
    #     sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../../snu_idim_3dp")) )
    #     from Automate3DP import Automate3DP

    #     device_name = 'printer0'
    #     device_class = Automate3DP(device_name)
    #     hub_device_part = DeviceHUB(device_name=device_name, device_class=Automate3DP(device_name))
    
    # elif mode == 'manager':
    #     device_name = 'printer0'
    #     device_class = None

    #     rospy.init_node('this_node_is_not_necessary_for_real_implementation')
    #     hub_manager_part = DeviceHUB(device_name=device_name, device_class=None);   sleep(3)

    #     cmd_dict = {'connection': True}
    #     hub_manager_part.manager_sendCommand(cmd_dict);                             sleep(10)

    #     status = hub_manager_part.manager_getStatus()
    #     print("[INFO] Current 'status_dict': \n{}".format(status))