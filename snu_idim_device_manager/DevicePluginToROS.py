#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
from threading import Thread
from time import sleep
import json

import rospy
from std_msgs.msg import String



class DevicePluginToROS:
    '''
    Class: DevicePluginToROS

    Descriptions:
        1. ROS node for a single device
        2. Node name: "{device_name}"
        3. Publish topic: "{device_name}/status" 
            - data format: python dict -> json -> string -> ROS String
        4. Subscribe topic: "{device_name}/command" 
            - data format: ROS String -> string -> json -> python dict
    '''
    def __init__(self, device_name='device_name', device_class=None):
        self.device_name = device_name
        self.device_status = dict()

        ## Node for device
        if device_class != None:
            self.device_class = device_class

            # rospy.init_node(self.device_name)
            self.device_status_publisher = rospy.Publisher("{}/status".format(self.device_name), String, queue_size=1)
            self.manager_cmd_publisher = rospy.Publisher("{}/command".format(self.device_name), String, queue_size=1)
            rospy.Subscriber('{}/command'.format(self.device_name), String, self.device_cmd_cb, queue_size=1)
            print("[INFO] ROS initialized for device ({})".format(self.device_name))
            
            thread_1 = Thread(target=self.device_runNode)
            thread_1.start()
            # self.device_runNode()
        
        ## Node for execution manager
        else:
            self.manager_cmd_publisher = rospy.Publisher("{}/command".format(self.device_name), String, queue_size=1)
            rospy.Subscriber('{}/status'.format(self.device_name), String, self.manager_status_cb, queue_size=1)
            print("[INFO] ROS initialized for device manager ({})".format(self.device_name))


    def device_publishStatus(self):
        # self.device_status = self.device_class.updateStatus()
        self.device_status = self.device_class.status
        # print('test', self.device_status)
        msg_json = json.dumps(self.device_status)
        self.device_status_publisher.publish(msg_json)
    

    def device_cmd_cb(self, msg):
        cmd_dict = json.loads(msg.data)
        print("[INFO] Received 'command_dict': \n{}".format(cmd_dict))
        self.device_class.command(cmd_dict)


    def device_runNode(self):
        while True:
            self.device_publishStatus()
            sleep(1.0)


    def manager_status_cb(self, msg):
        self.device_status = json.loads(msg.data)
        # print("[INFO] Received 'status_dict': \n{}".format(self.device_status))


    def sendCommand(self, cmd_dict):
        cmd_json = json.dumps(cmd_dict)
        self.manager_cmd_publisher.publish(cmd_json)


    def getStatus(self):
        # print("[INFO] Current 'status_dict': \n{}".format(self.device_status))
        return self.device_status




if __name__ == '__main__':

    # mode = 'manager'
    mode = 'device'

    if mode == 'device':
        sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_3dp")) )
        from DeviceClass_3DP import DeviceClass_3DP

        device_name = 'printer0'
        hub_device_part = DevicePluginToROS(device_name=device_name, device_class=DeviceClass_3DP(device_name))
    
    elif mode == 'manager':
        device_name = 'printer0'
        device_class = None

        rospy.init_node('this_node_is_not_necessary_for_real_implementation')
        hub_manager_part = DevicePluginToROS(device_name=device_name, device_class=None);     sleep(3)

        cmd_dict = {'connection': True}
        hub_manager_part.sendCommand(cmd_dict);     sleep(10)

        status = hub_manager_part.getStatus()
        print("[INFO] Current 'status_dict': \n{}".format(status))