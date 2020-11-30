#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, time
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from autoRun import *

import rospy
from std_msgs.msg import String, Int32MultiArray



class auto3DP:
    def __init__(self, folder_dir='src'):
        rospy.init_node('printer_node')
        rospy.Subscriber('printer/command', String, self.cmd_callback)
        self.printer_status_pub = rospy.Publisher("printer/status", String, queue_size=1)

        self.printer_status = [0, 0, 0, 0]
        self.printer_cmd    = [0, 0, 0, 0]

        self.autoRun = idimAutomation(folder_dir)

        print('[DEBUG] Node initialized !!!')
    

    def cmd_callback(self, msg):
        cmd_list = msg.data.split(',')
        print("[DEBUG] 3D printers command list: {}".format(cmd_list))

        for i in range(len(cmd_list)):
            cmd = int(cmd_list[i])
            if cmd != 0 and self.printer_status[i] == 0:
                try:
                    print("[DEBUG] Printer #{} in action !!! (command: {})".format(i, cmd))
                    self.printer_status[i] = 1
                    self.autoRun.execute('3DP_{}_{}.txt'.format(i, cmd))
                    print("[DEBUG] Printer #{} task is done !!! (command: {})".format(i, cmd))
                except:
                    print("[ERROR] Script '{}' does not exist !!!".format('3DP_{}_{}.txt'.format(i, cmd)))
                self.printer_status[i] = 0
            elif self.printer_status[i] == 1:
                print("[DEBUG] Printer #{} is busy now ... (cannot assign a new task)")


    def pub_status(self):
        s = self.printer_status
        self.printer_status_pub.publish('{},{},{},{}'.format(s[0], s[1], s[2], s[3]))



if __name__ == "__main__":

    auto3DP = auto3DP()

    print("[DEBUG] 3DP Automation Started !!!")

    ## Automation program setting
    folder_dir = "src"
    script1 = '3DP_1_start.txt'
    script2 = '3DP_1_stop.txt'
    script3 = '3DP_2_start.txt'
    script4 = '3DP_2_stop.txt'

    time.sleep(1.0)

    ## Create an instance (initialize)
    auto3dp = idimAutomation(folder_dir)

    ## Start automation
    # auto3dp.execute(script1)
    # auto3dp.execute(script2)
    # auto3dp.execute(script3)
    # auto3dp.execute(script4)
    
    while True:
        auto3DP.pub_status()
        rospy.sleep(0.1)
        pass