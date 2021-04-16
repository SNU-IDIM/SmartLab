#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
cd /
cd dev
sudo chown hong_jetson ttyUSB0

'''
import rospy
import os, sys
import time
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),'../snu_idim_device_manager')) )

from DevicePluginToROS import DevicePluginToROS
from DeviceClass_OMM import DeviceClass_OMM


if __name__ == "__main__":
    device_name = 'MS'
    rospy.init_node(device_name)
    measurement_node = DevicePluginToROS(device_name=device_name, device_class=DeviceClass_OMM(device_name, port_='/dev/ttyUSB0'))


    specimen_name = 'default'

    time.sleep(0.2)
    cmd_dict = dict()
    cmd_dict['connection'] = specimen_name
    measurement_node.sendCommand(cmd_dict)

    # time.sleep(5)
    # cmd_dict['wake'] = specimen_name
    # del(cmd_dict['connection'])
    # measurement_node.sendCommand(cmd_dict)
    

    # time.sleep(10.0)

    # while True:
    #     if measurement_node.getStatus()['status'] == 'Idle':
    #         print('\n measure debugging \n')
    #         del(cmd_dict['connection'])
            
    #         cmd_dict['measure_dimension'] = specimen_name
    #         measurement_node.sendCommand(cmd_dict)
    #         time.sleep(2)
    #         break
    #     else:
    #         continue