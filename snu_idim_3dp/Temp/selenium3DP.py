#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
from time import sleep
import json


from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.support.ui import Select, WebDriverWait
from selenium.common.exceptions import TimeoutException
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.common.by import By



import rospy
from std_msgs.msg import String





class DeviceROSNode:
    '''
    Class: DeviceROSNode

    Descriptions:
        1. ROS node for a single device
        2. Initialize ROS node with a given 'device_name'
        3. Publish device status (format: python dict -> json -> string -> ROS String)
        4. Subscribe a command and pass it to the device-specified class (ROS String -> string -> json -> python dict)

    '''
    def __init__(self, device_name):
        rospy.init_node(device_name)
        print("ROS initialized")
        self.device = Automate3DP(ID=0)
        self.device_status = dict()
        self.device_status_publisher = rospy.Publisher("{}/status".format(device_name), String, queue_size=1)
        rospy.Subscriber('{}/command'.format(device_name), String, self.cmd_cb, queue_size=1)


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

    ros_node = DeviceROSNode('printer0') 
    ros_node.runNode()
