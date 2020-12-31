#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
from threading import Thread
from time import sleep
import json

import rospy
from std_msgs.msg import String




if __name__ == '__main__':


    device_name = 'instron'

    rospy.init_node('this_node_is_not_necessary_for_real_implementation')
    pub_status = rospy.Publisher("{}/status".format(device_name), String, queue_size=1)

    while True:
        msg_json = json.dumps({'status': 'Ready'})
        pub_status.publish(msg_json)
        rospy.sleep(5.0)
        msg_json = json.dumps({'status': 'Idle'})
        pub_status.publish(msg_json)
        rospy.sleep(5.0)
        msg_json = json.dumps({'status': 'data_sent'})
        pub_status.publish(msg_json)
        rospy.sleep(5.0)
        msg_json = json.dumps({'status': 'Idle'})
        pub_status.publish(msg_json)
        rospy.sleep(5.0)
