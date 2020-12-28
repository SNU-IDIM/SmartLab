#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, json
from time import sleep
from copy import deepcopy
from threading import Thread


import rospy, actionlib
from std_msgs.msg import String
from syscon_msgs.msg import *

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/imp")) )
from IDIM_framework import *

from DevicePluginToROS import DevicePluginToROS

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_3dp")) )
from DeviceClass_3DP import DeviceClass_3DP


class DeviceManager():
    def __init__(self):
        self.device_dict = dict()
        self.printer_list_idle     = []
        self.printer_list_initializing = []
        self.printer_list_printing = []
        self.printer_list_finished = []  ## ***
        self.printer_list_robot_done = []

        self.specimen_ready_list = []  ## ***

        self.test_ready_list = []  ## ***

        self.client = actionlib.SimpleActionClient('/R_001/WAS', WorkFlowAction)
        self.client.wait_for_server(timeout=rospy.Duration(1))
        self.amr = WorkFlowGoal()
        self.amr_param = [Param('max_trans_vel','float','0.3'),
                          Param('max_rot_vel','float','0.25'), 
                          Param('xy_goal_tolerance','float','0.20'),
                          Param('yaw_goal_tolerance','float','0.05')]
        self.amr.work = []
        self.amr.work_id = 'amr' # default: "snu_state_machine"
        self.amr.loop_flag = 1     # default: 1 (no repeat)
        
        self.thread_1 = Thread(target=self.manager3DP)
        self.thread_1.start()

        self.thread_2 = Thread(target=self.managerCobot)
        self.thread_2.start()


    def __del__(self):
        self.thread_1.terminate()
        self.thread_2.terminate()
        pass


    def addDevice(self, device_name, device_class=None):
        self.device_dict[device_name] = DevicePluginToROS(device_name=device_name, device_class=device_class)
        print("[DEBUG] '{}' is added to DeviceManager".format(device_name))

    def printStatus(self, status_dict):
        key_list = status_dict.keys()
        value_list = status_dict.values()
        print("\n=======================================================")
        for i in range(len(key_list)):
            print("[DEBUG - {}] {}: {}".format(status_dict['device_name'], key_list[i], value_list[i]))

    def moveAMR(self, target_pose, spot_name="default", hold_time=0.0):
        print("[AMR] AMR Start Moving ... (target pose = {})".format(target_pose))
        work = Action(SYSCON_WAYPOINT, target_pose, self.amr_param)
        self.amr.work.append(work)
        self.amr.work_id = spot_name
        self.client.send_goal(self.amr)
        self.client.wait_for_result()
        print("[AMR] Arrived at [{}] !!!".format(target_pose))
        self.amr.work = []
        rospy.sleep(hold_time)

    def managerCobot(self):
        printer_number = 1
        sensor_number = 1
        # AMR 이동 (3DP-N)
        task_get_bed = [ACTION_HOME, ACTION_TOOLCHANGE_1_ATTACH, TASK_3DP_BED_OUT - printer_number, ACTION_HOME]
        task_detach_specimen = [ACTION_TOOLCHANGE_1_DETACH, ACTION_TOOLCHANGE_2_ATTACH, ACTION_HOME, TASK_DETACH_SPECIMEN, TASK_SEARCH_PICK_SPECIMEN, TASK_SEPCIMEN_TO_CENTER]
        task_attach_sensor = [TASK_ADHESIVE_SAVER_OUT, TASK_SPECIMEN_TO_LEFT, TASK_ADHESIVE_DROP, TASK_SPECIMEN_TO_RIGHT, TASK_ADHESIVE_SAVER_IN, TASK_ATTACH_SENSOR + sensor_number]
        task_specimen_to_rack = [TASK_SPECIMEN_READY, TASK_PICK_PLACE_RACK, TASK_RACK_ALIGN]
        task_return_bed = [ACTION_TOOLCHANGE_2_DETACH, ACTION_TOOLCHANGE_1_ATTACH, TASK_3DP_BED_IN + printer_number, ACTION_HOME]
        # AMR 이동 (인장시험기)
        # 실험 시작 (인장시험기 command)
        
        # test_task = [ACTION_HOME, ACTION_TOOLCHANGE_1_DETACH, ACTION_TOOLCHANGE_2_ATTACH]
        test_task = []
        robot_task_queue = [3050, -3050, 3050, -3050]


        while True:
            try:
                # self.printStatus(self.device_dict['R_001/cobot'].getStatus())
                if self.device_dict['R_001/cobot'].getStatus()['status'] == 'Standby':
                    task_next = test_task.pop(0)
                    self.device_dict['R_001/cobot'].sendCommand({"command": '{}'.format(task_next)})
                    # print("[DEBUG] Robot task queue: {}".format(test_task))

                # print(self.device_dict['R_001/cobot'].getStatus())
            except:
                print("[ERROR] Collaborative Robot is not connected yet !!!")

            sleep(3.0)


    def manager3DP(self):
        # printing_queue = ['test1', 'test2', 'test3', 'test1', 'test2', 'test3']
        printing_queue = []
        while True:
            n_printer = 0
            id_list_idle = []
            id_list_initializing= []
            id_list_printing = []
            id_list_finished = []
            
            for i in range(len(self.device_dict)):
                n_printer += 1 if self.device_dict[self.device_dict.keys()[i]].getStatus()['device_type'] == '3D Printer' else n_printer

                if self.device_dict[self.device_dict.keys()[i]].getStatus()['device_type'] == '3D Printer':
                    printer_id = self.device_dict[self.device_dict.keys()[i]].getStatus()['device_name']
                    printer_status = self.device_dict[self.device_dict.keys()[i]].getStatus()['status']

                    if printer_status.find('Idle') != -1:
                        id_list_idle.append(printer_id)
                    elif printer_status.find('Initializing') != -1:
                        id_list_initializing.append(printer_id)
                    elif printer_status.find('Printing') != -1:
                        id_list_printing.append(printer_id)
                    elif printer_status.find('Done') != -1:
                        id_list_finished.append(printer_id)

            self.printer_list_idle = id_list_idle
            self.printer_list_initializing = id_list_initializing
            self.printer_list_printing = id_list_printing
            self.printer_list_finished = id_list_finished

            printer_list_robot_done = self.printer_list_robot_done
            for printer_id in printer_list_robot_done:
                # print(printer_id)
                self.printer_list_finished.remove(printer_id)
                self.printer_list_idle.append(printer_id)
                self.printer_list_robot_done.remove(printer_id)
                self.device_dict[printer_id].sendCommand({'status': 'Idle'})


            # print("[INFO - DeviceManager] # of 3D Printers: {}".format(n_printer))
            # print("[INFO - DeviceManager] # of 3D Printers in Idle: {}".format(len(self.printer_list_idle)), self.printer_list_idle)
            # print("[INFO - DeviceManager] # of 3D Printers in Initializing: {}".format(len(self.printer_list_initializing)), self.printer_list_initializing)
            # print("[INFO - DeviceManager] # of 3D Printers in Printing: {}".format(len(self.printer_list_printing)), self.printer_list_printing)
            # print("[INFO - DeviceManager] # of 3D Printers in Finished: {}".format(len(self.printer_list_finished)), self.printer_list_finished)
            # print("[INFO - DeviceManager] # of 3D Printers in Robot Job Done: {}".format(len(self.printer_list_robot_done)), self.printer_list_robot_done)


            # print("[INFO - DeviceManager] Printing queue: {}".format(printing_queue))
            for printer_in_idle in self.printer_list_idle:
                try:
                    print_next = printing_queue.pop(0)
                    self.device_dict[printer_in_idle].sendCommand({'print': print_next})
                except:
                    # print("[INFO - DeviceManager] Printing queue: empty !!!")
                    pass
            


            sleep(3.0)


if __name__ == '__main__':

    rospy.init_node('DeviceManager')

    manager = DeviceManager()
    # manager.addDevice('printer1', DeviceClass_3DP(device_name='printer1', ip_='192.168.60.101', port_='5001'))
    # manager.addDevice('printer2', DeviceClass_3DP(device_name='printer2', ip_='192.168.60.101', port_='5002'))
    # manager.addDevice('printer3', DeviceClass_3DP(device_name='printer3', ip_='192.168.60.101', port_='5003'))
    # manager.addDevice('printer4', DeviceClass_3DP(device_name='printer4', ip_='192.168.60.101', port_='5004'))
    manager.addDevice('R_001/cobot', device_class=None)

    sleep(5.0)

    manager.moveAMR(spot_name='Instron', target_pose=[3.016, -3.244, -1.622], hold_time=0.0)

    # # sleep(5.0)
    # # manager.device_dict['R_001/cobot'].sendCommand({"command": '0'})


    # sleep(15.0)

    # for i in range(3):
    #     manager.device_dict['printer{}'.format(i)].sendCommand({"connection": True})
    # while True:
    #     sleep(3.0)
    #     if len(manager.printer_list_finished) != 0:
    #         print("[DEBUG] Robot Working................")
    #         sleep(10.0)
    #         manager.printer_list_robot_done.append(manager.printer_list_finished[0])
    #     # if manager.device_dict['printer0'].device_status['status'].find('Idle') != -1:
    #     #     hmanagerub.device_dict['printer0'].sendCommand({'print': '201122_feedrate_test'})