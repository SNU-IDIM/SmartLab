#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os, sys
import time
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
    cmd_dict['setup'] = 'test5'

    instron_node.sendCommand(cmd_dict)  #   time.sleep(3)


    '''
    while True:
        # print("3")
        if instron_node.getStatus()['status'] == 'Ready':
            print("execute debugging")
            cmd_dict['execute'] = 'test1'
            del(cmd_dict['setup'])
            instron_node.sendCommand(cmd_dict)
            break
        else:
            continue
    while True:
        if instron_node.getStatus()['status'] =='Idle':
            print("result debugging")
            cmd_dict['result'] = 'specimenc'
            del(cmd_dict['execute'])
            # del(cmd_dict['setup'])
            instron_node.sendCommand(cmd_dict)
            break

        else:
            continue
    '''

    # while True:
    #     if instron_node.getStatus()['status'] == 'Ready':
    #         print("execute debugging")
    #         cmd_dict['execute'] = 'test5'
    #         del(cmd_dict['setup'])
    #         instron_node.sendCommand(cmd_dict)
    #         # breap00-oo00000099e:
    #         break
    #     else:
    #         continue
    
    # while True:
    #     if instron_node.getStatus()['status'] =='Idle':
    #         print("result debugging")
    #         cmd_dict['result'] = 'test5'
    #         del(cmd_dict['execute'])
    #         instron_node.sendCommand(cmd_dict)
    #         break
        
    #     else:
    #         continue
