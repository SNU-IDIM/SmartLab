#! /usr/bin/env python
# -*- coding: utf-8 -*-

## sudo usermod -a -G tty threpsilon

import rospy
import os, sys
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_device_manager")) )

from DevicePluginToROS import DevicePluginToROS
from DeviceClass_MS_dimension import measureStation


if __name__ == "__main__":
    device_name = 'MS_station'

    measurement_node = DevicePluginToROS(device_name=device_name, device_class=measureStation(device_name, port_='/dev/ttyUSB0'))


    specimen_name = 'base'

    rospy.sleep(5)
    cmd_dict = dict()
    cmd_dict['connection'] = specimen_name
    measurement_node.sendCommand(cmd_dict)
    

    while True:
        if measurement_node.getStatus()['status'] == 'Ready':
            print('measure debugging')
            del(cmd_dict['connection'])
            cmd_dict['measure_dimension'] = specimen_name
            measurement_node.sendCommand(cmd_dict)
            break
        else:
            continue
    
    while True:
        if measurement_node.getStatus()['status'] == 'Ready':
            del(cmd_dict['measure_dimension'])
            cmd_dict['save_result'] = specimen_name
            measurement_node.sendCommand(cmd_dict)
            break
        else:
            continue
    
            
        