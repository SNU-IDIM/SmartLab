#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
cd /
cd dev
sudo chown threpsilon ttyUSB0
'''
import rospy
import os, sys
import time
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),'../snu_idim_device_manager')) )

from DevicePluginToROS import DevicePluginToROS
from DeviceClass_MS_dimension import DeviceClass_OMM


if __name__ == "__main__":
    device_name = 'MS'

    measurement_node = DevicePluginToROS(device_name=device_name, device_class=DeviceClass_OMM(device_name, port_='/dev/ttyUSB0'))


    specimen_name = '20210302t1'

    rospy.sleep(5)
    cmd_dict = dict()
    cmd_dict['connection'] = specimen_name
    measurement_node.sendCommand(cmd_dict)

    

    # while True:
    #     if measurement_node.getStatus()['connection'] == True:
    #         print('\n debugging start \n')
    #         del(cmd_dict['connection'])
    #         cmd_dict['measure_thickness'] = specimen_name
    #         measurement_node.sendCommand(cmd_dict)
    #         break
    #     else:
    #         continue

    # while True:
    #     if measurement_node.getStatus()['status'] == 'Idle':
    #     # if measurement_node.getStatus()['connection'] == True:
    #         print('\n measure debugging \n')
    #         del(cmd_dict['measure_thickness'])
    #         # del(cmd_dict['connection'])
            
    #         cmd_dict['measure_dimension'] = specimen_name
    #         measurement_node.sendCommand(cmd_dict)
    #         time.sleep(2)
    #         break
    #     else:
    #         continue
    
    # while True:
    #     if measurement_node.getStatus()['status'] == 'Idle':
    #         del(cmd_dict['measure_dimension'])
    #         cmd_dict['save_result'] = specimen_name
    #         measurement_node.sendCommand(cmd_dict)
    #         break
    #     else:
    #         continue
    
            
        