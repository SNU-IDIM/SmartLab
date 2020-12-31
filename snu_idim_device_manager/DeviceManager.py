#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, json, time
from time import sleep
from copy import deepcopy
from threading import Thread
import zmq


import rospy, actionlib
from std_msgs.msg import String
from syscon_msgs.msg import *

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/imp")) )
from IDIM_framework import *

from DevicePluginToROS import DevicePluginToROS

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_3dp")) )
from DeviceClass_3DP import DeviceClass_3DP

# sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_instron")) )
# from DeviceClass_Instron import DeviceClass_Instron


class DeviceManager():

    def __init__(self, port_=5555):
        ## ZMQ: ROS(server) <-> Python(client)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("tcp://*:{}".format(port_))

        ## Device dict (list of devices to handle with ID)
        self.device_dict = dict() # for device manager
        self.device_info = dict() # to send data to client via ZMQ
        
        ## for 3DP Manager
        self.printer_list_idle         = list()
        self.printer_list_initializing = list()
        self.printer_list_printing     = list()
        self.printer_list_finished     = list()  ## ***
        self.printer_list_robot_done   = list()

        self.specimen_ready_list = list()  ## ***
        self.test_ready_list     = list()  ## ***

        ## for Cobot Manager
        self.cobot_task_queue = list()

        ## for AMR Manager
        self.client = actionlib.SimpleActionClient('/R_001/WAS', WorkFlowAction)
        self.client.wait_for_server(timeout=rospy.Duration(1))
        self.amr = WorkFlowGoal()
        self.amr_param = [Param('max_trans_vel','float','0.3'),
                          Param('max_rot_vel','float','0.25'), 
                          Param('xy_goal_tolerance','float','0.20'),
                          Param('yaw_goal_tolerance','float','0.05')]
        self.amr.work = []
        self.amr.work_id = 'amr'
        self.amr.loop_flag = 1  # default: 1 (no repeat)
        
        ## 3DP Manager thread
        self.thread_1 = Thread(target=self.manager3DP)
        self.thread_1.start()

        ## Cobot Manager thread
        self.thread_2 = Thread(target=self.executionManager)
        self.thread_2.start()

        ## Refreshing 'device_info' thread (for Client)
        self.thread_3 = Thread(target=self.refreshDeviceInfo)
        self.thread_3.start()

        ## ZMQ Server thread
        self.thread_4 = Thread(target=self.zmq_server)
        self.thread_4.start()



    def __del__(self):
        self.thread_1.terminate()
        self.thread_2.terminate()
        self.thread_3.terminate()
        self.thread_4.terminate()
        pass

    

    def refreshDeviceInfo(self):
        while True:
            print("[DEBUG] Device List: {}".format(self.device_dict.keys()))
            self.device_info = dict()

            keys = self.device_dict.keys()
            values = self.device_dict.values()

            for i in range(len(keys)):
                self.device_info[keys[i]] = self.device_dict[keys[i]].getStatus()

            # print("[DEBUG] Device information updated !!! (Devices: {})".format(self.device_info.keys()))
            sleep(1.0)

    

    def zmq_server(self):
        while True:
            request = json.loads(self.socket.recv())
            try:
                print("[DEBUG] Client requested status of devices ({})".format(request['status']))
                self.socket.send_string(json.dumps(self.device_info))
            except:
                pass

            sleep(1.0)



    def addDevice(self, device_name, device_class=None):
        self.device_dict[device_name] = DevicePluginToROS(device_name=device_name, device_class=device_class)
        print("[DEBUG] '{}' is added to DeviceManager".format(device_name))



    def printStatus(self, status_dict):
        key_list = status_dict.keys()
        value_list = status_dict.values()

        print("\n=======================================================")
        for i in range(len(key_list)):
            print("[DEBUG - {}] {}: {}".format(status_dict['device_name'], key_list[i], value_list[i]))



    def executeAMR(self, target_pose, spot_name="default", hold_time=0.0):
        print("[AMR] AMR Start Moving ... (target pose = {})".format(target_pose))
        work = Action(SYSCON_WAYPOINT, target_pose, self.amr_param)
        self.amr.work.append(work)
        self.amr.work_id = spot_name
        self.client.send_goal(self.amr)
        self.client.wait_for_result()
        print("[AMR] Arrived at [{}] !!!".format(target_pose))
        self.amr.work = []
        rospy.sleep(hold_time)
    


    def executeCobot(self, robot_task_queue, mode='real'):
        if mode == 'real':
            while len(robot_task_queue) != 0:
                if self.device_dict['R_001/cobot'].getStatus()['status'] == 'Standby':
                    print('1')
                    next_task = robot_task_queue.pop(0)
                    print('2')
                    self.device_dict['R_001/cobot'].sendCommand({'command': next_task})
                    print('3')
                    print("[Cobot] Robot task queue: {}".format(robot_task_queue))
                    print('4')
                    sleep(3.0)
        elif mode == 'debug':
            while len(robot_task_queue) != 0:
                next_task = robot_task_queue.pop(0)
                print("[Cobot] Robot task queue: {}".format(robot_task_queue))
                sleep(0.2)


    
    def executeInstron(self, subject_name):
        self.device_dict['instron'].sendCommand({'setup': subject_name})

        while True:
            if self.device_dict['instron'].getStatus()['status'] == 'Ready':
                self.device_dict['instron'].sendCommand({'execute': 'specimen1'})
                print("[Instron] Test Start !!!")
                break




    def makeRobotTaskQueue(self, printer_id):
        printer_number = int(printer_id.split('printer')[1])
        sensor_number = printer_number  ## 임시 (로봇쪽 sensor number 작업 필요)

        task_get_bed = [ACTION_HOME, ACTION_TOOLCHANGE_1_ATTACH, TASK_3DP_BED_OUT - printer_number, ACTION_HOME]
        task_detach_specimen = [ACTION_TOOLCHANGE_1_DETACH, ACTION_TOOLCHANGE_2_ATTACH, ACTION_HOME, TASK_DETACH_SPECIMEN, TASK_SEPCIMEN_TO_CENTER]
        task_specimen_to_rack = [TASK_SPECIMEN_TO_RACK, TASK_RACK_ALIGN]
        task_return_bed = [ACTION_TOOLCHANGE_2_DETACH, ACTION_TOOLCHANGE_1_ATTACH, TASK_3DP_BED_IN + printer_number, ACTION_HOME]
        task_feed_specimen = [ACTION_TOOLCHANGE_1_DETACH, ACTION_TOOLCHANGE_2_ATTACH, ACTION_HOME, TASK_SPECIMEN_FROM_RACK, ACTION_HOME, TASK_INSTRON_SEARCH]
        # task_get_bed = [ACTION_HOME, ACTION_TOOLCHANGE_1_ATTACH, TASK_3DP_BED_OUT - printer_number, ACTION_HOME]
        # task_detach_specimen = [ACTION_TOOLCHANGE_1_DETACH, ACTION_TOOLCHANGE_2_ATTACH, ACTION_HOME, TASK_DETACH_SPECIMEN, TASK_SEPCIMEN_TO_CENTER]
        # task_attach_sensor = [TASK_ADHESIVE_SAVER_OUT, TASK_SPECIMEN_TO_LEFT, TASK_ADHESIVE_DROP, TASK_SPECIMEN_TO_RIGHT, TASK_ADHESIVE_SAVER_IN, TASK_ATTACH_SENSOR + sensor_number]
        # task_specimen_to_rack = [TASK_SPECIMEN_READY, TASK_PICK_PLACE_RACK, TASK_RACK_ALIGN]
        # task_return_bed = [ACTION_TOOLCHANGE_2_DETACH, ACTION_TOOLCHANGE_1_ATTACH, TASK_3DP_BED_IN + printer_number, ACTION_HOME]
        robot_task_queue = task_get_bed + task_detach_specimen + task_specimen_to_rack + task_return_bed + task_feed_specimen

        return robot_task_queue


    def executionManager(self):
        while True:
            try:
                print("[Execution Manager] Print done list: {}".format(self.printer_list_finished))
                printer_id = self.printer_list_finished.pop(0)

                ## 1. AMR 이동 (printer_id)
                print("[Execution Manager] AMR moving... (target: {})".format(printer_id))
                # self.executeAMR(spot_name=printer_id, target_pose=[3.016, -3.244, -1.622], hold_time=0.0) # target_pose 수정 작업 필요

                ## 2. 협동로봇 시편 준비 작업 (printer_id)
                print("[Execution Manager] Robot task start !!! (printer: {})".format(printer_id))
                robot_task_queue = self.makeRobotTaskQueue(printer_id)
                print(robot_task_queue)
                # robot_task_queue = [3050, -3050, 3050, -3050]  # for debug
                self.executeCobot(robot_task_queue, mode='real')
                self.printer_list_robot_done.append(printer_id)
                print("[Execution Manager] Robot task done !!! (printer: {})".format(printer_id))

                ## 3. 인장시험 수행 (subject_id)
                subject_id = self.device_info[printer_id]['subject_name']
                print("[Execution Manager] Experiment start !!! (subject: {})".format(subject_id))
                self.executeInstron(subject_id)

            except:
                print("[Execution Manager] Error !!!")
                pass

            sleep(3.0)



    def manager3DP(self):
        printing_queue = ['test1', 'test2', 'test3', 'test1', 'test2', 'test3']
        # printing_queue = []
        while True:
            
            for i in range(len(self.device_dict)):
                try:
                    device_id = self.device_dict.keys()[i]
                    device_status = self.device_dict[self.device_dict.keys()[i]].getStatus()
                    device_type = device_status['device_type']

                    if device_type == '3D Printer':

                        ## 3DP status: Done -> Idle (if robot task is done with that 3DP)
                        try:
                            idx = self.printer_list_robot_done.index(device_id)
                            self.printer_list_robot_done.pop(idx)
                            self.device_dict[device_id].sendCommand({'status': 'Idle'})
                            device_status['status'] = 'Idle'
                            print("[3DP] {} status: Done -> Idle".format(device_id))
                        except:
                            pass

                        ## Print a new subject from printing queue
                        if device_status['status'].find('Done') != -1:
                            try:
                                self.printer_list_finished.index(device_id)
                            except:
                                self.printer_list_finished.append(device_id)
                        else:
                            try:
                                idx = self.printer_list_finished.index(device_id)
                                self.printer_list_finished.pop(idx)
                            except:
                                pass

                        ## Print a new subject from printing queue
                        if device_status['status'].find('Idle') != -1:
                            try:
                                print_next = printing_queue.pop(0)
                                self.device_dict[device_id].sendCommand({'print': print_next})
                                print("[3DP] {} status: Idle -> Printing {}".format(device_id, print_next))
                            except:
                                print("[3DP] Printing queue is empty !!!")
                    
                    # print("\n[DEBUG] 3DP finished: {}".format(self.printer_list_finished))
                    # print("[DEBUG] 3DP robot done: {}".format(self.printer_list_robot_done))
                    # print("[DEBUG] Subject name: {}".format(device_status['subject_name']))
                except:
                    pass
                    
            sleep(3.0)




if __name__ == '__main__':

    rospy.init_node('DeviceManager')

    manager = DeviceManager()
    # manager.addDevice('printer1', DeviceClass_3DP(device_name='printer1', port_='5000'))
    


    manager.addDevice('R_001/cobot', device_class=None)
    manager.addDevice('instron')
    manager.addDevice('printer1', DeviceClass_3DP(device_name='printer1', ip_='192.168.60.101', port_='5001'))
    # manager.addDevice('printer2', DeviceClass_3DP(device_name='printer2', ip_='192.168.60.101', port_='5002'))
    # manager.addDevice('printer3', DeviceClass_3DP(device_name='printer3', ip_='192.168.60.101', port_='5003'))
    # manager.addDevice('printer4', DeviceClass_3DP(device_name='printer4', ip_='192.168.60.101', port_='5004'))


    # sleep(5.0)
    # manager.executeAMR(spot_name='Instron', target_pose=[3.016, -3.244, -1.622], hold_time=0.0)

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