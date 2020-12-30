#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import os, sys
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_device_manager")) )

from DevicePluginToROS import DevicePluginToROS
from DeviceClass_Instron import DeviceClass_Instron


if __name__ == "__main__":
    device_name = 'instron'
    rospy.init_node(device_name)
    instron_node = DevicePluginToROS(device_name=device_name, device_class=DeviceClass_Instron(device_name=device_name))
    # '''
    rospy.sleep(5.0)
    cmd_dict = dict()
    cmd_dict['setup'] = 'specimenb'

    instron_node.sendCommand(cmd_dict);     time.sleep(10)

    while True:

        if instron_node.getStatus()['status'] == 'Ready':
            print("execute debugging")
            cmd_dict['execute'] = 'specimenb'
            del(cmd_dict['setup'])
            instron_node.sendCommand(cmd_dict)
            break

    while True:
        if instron_node.getStatus()['status'] =='Idle':
            print("result debugging")
            cmd_dict['result'] = 'specimenb'
            del(cmd_dict['execute'])
            instron_node.sendCommand(cmd_dict)
            break

        else:
            continue
            
    # '''