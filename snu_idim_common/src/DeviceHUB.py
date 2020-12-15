#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
from time import sleep
import json

import rospy
from std_msgs.msg import String



class DeviceHUB:
    '''
    Class: DeviceHUB

    Descriptions:
        1. ROS node for a single device
        2. Node name: "{device_name}"
        3. Publish topic: "{device_name}/status" 
            - data format: python dict -> json -> string -> ROS String
        4. Subscribe topic: "{device_name}/command" 
            - data format: ROS String -> string -> json -> python dict
    '''
    def __init__(self, device_name, device_class):
        self.device = device_class
        self.device_status = dict()

        rospy.init_node(device_name)
        self.device_status_publisher = rospy.Publisher("{}/status".format(device_name), String, queue_size=1)
        rospy.Subscriber('{}/command'.format(device_name), String, self.cmd_cb, queue_size=1)
        print("[INFO] ROS initialized for device ({})".format(device_name))
        
        self.runNode()


    def publishStatus(self):
        self.device_status = self.device.updateStatus()
        msg_json = json.dumps(self.device_status)
        self.device_status_publisher.publish(msg_json)
    

    def cmd_cb(self, msg):
        cmd_dict = json.loads(msg.data)
        print("[INFO] Received 'command_dict': \n{}".format(cmd_dict))
        self.device.command(cmd_dict)


    def runNode(self):
        while True:
            self.publishStatus()
            sleep(1.0)





if __name__ == '__main__':

	sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../../snu_idim_3dp")) )
	from Automate3DP import Automate3DP
	
	device_name='printer0'
	
	ros_node = DeviceHUB(device_name=device_name, device_class=Automate3DP(device_name))