#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import *

            
class CommTest():
    def __init__(self, ros_node_name="snu_state_machine", work_id="default", loop_number=1):
        rospy.init_node("communication_test", anonymous=True)
        self.pnp_pub = rospy.Publisher("/main_pc", Float64, queue_size=1)


if __name__ == '__main__':
    test = CommTest()
    test.pnp_pub.publish(10)
 
    while not rospy.is_shutdown():
        test.pnp_pub.publish(10)
        rospy.sleep(1)
        pass
