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

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_doe")) )
from TestDesigner import TestDesigner

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/imp")) )
from IDIM_framework import *

from DevicePluginToROS import DevicePluginToROS

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_3dp")) )
from DeviceClass_3DP import DeviceClass_3DP

# sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_instron")) )
# from DeviceClass_Instron import DeviceClass_Instron


class TestManager():
    def __init__(self, test_setting, ip='localhost'):
        self.test_designer = TestDesigner(test_setting=test_setting, ip=ip)
        self.test_id_list = self.test_designer.sendDOE()
    
    def getTestIDs(self):
        print("[DEBUG - TestManager] Test ID list: \n{}".format(self.test_id_list))
        return self.test_id_list



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
        self.printing_queue            = list()
        self.printer_list_idle         = list()
        self.printer_list_initializing = list()
        self.printer_list_printing     = list()
        self.printer_list_finished     = list()  ## ***
        self.printer_list_robot_done   = list()

        self.specimen_ready_list = list()  ## ***
        self.test_ready_list     = list()  ## ***

        ## for Cobot Manager
        self.cobot_task_queue = list()
        self.cobot_recent_work = None

        ## for Instron Manager
        self.instron_save_flag = False

        ## for AMR Manager
        # self.client = actionlib.SimpleActionClient('/R_001/WAS', WorkFlowAction)
        # self.client.wait_for_server(timeout=rospy.Duration(1))
        # self.amr = WorkFlowGoal()
        # self.amr_param = [Param('max_trans_vel','float','0.3'),
        #                   Param('max_rot_vel','float','0.25'), 
        #                   Param('xy_goal_tolerance','float','0.20'),
        #                   Param('yaw_goal_tolerance','float','0.05')]
        # self.amr.work = []
        # self.amr.work_id = 'amr'
        # self.amr.loop_flag = 1  # default: 1 (no repeat)
        
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


    '''#####################################################################################################
        Device manager related
    '''
    def addDevice(self, device_name, device_class=None):
        self.device_dict[device_name] = DevicePluginToROS(device_name=device_name, device_class=device_class)
        print("[DEBUG] '{}' is added to DeviceManager".format(device_name))


    def printStatus(self, status_dict):
        key_list = status_dict.keys()
        value_list = status_dict.values()

        print("\n=======================================================")
        for i in range(len(key_list)):
            print("[DEBUG - {}] {}: {}".format(status_dict['device_name'], key_list[i], value_list[i]))


    def refreshDeviceInfo(self):
        while True:
            # print("[DEBUG] Device List: {}".format(self.device_dict.keys()))
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
            print('GGGGGGGGGGGGGG')
            try:
                print("[DEBUG] Client requested status of devices ({})".format(request['status']))
                self.socket.send_string(json.dumps(self.device_info))
            except:
                pass
    

    '''#####################################################################################################
        3D Printer related
    '''
    def addPrintingQueue(self, printing_queue):
        self.printing_queue = printing_queue


    def manager3DP(self):
        while True:
            for i in range(len(self.device_dict)):
                try:
                    device_id = self.device_dict.keys()[i]
                    device_status = self.device_dict[self.device_dict.keys()[i]].getStatus()
                    device_type = device_status['device_type']

                    if device_type == 'Printer':

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
                                print_next = self.printing_queue.pop(0)
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


    '''#####################################################################################################
        Robot related
    '''
    def executeAMR(self, target_pose, spot_name="default", hold_time=0.0, debug=False):
        if debug == False:
            print("[AMR - Real mode] AMR Start Moving ... (target pose = {})".format(target_pose))
            work = Action(SYSCON_WAYPOINT, target_pose, self.amr_param)
            self.amr.work.append(work)
            self.amr.work_id = spot_name
            self.client.send_goal(self.amr)
            self.client.wait_for_result()
            print("[AMR - Real mode] Arrived at [{}] !!!".format(target_pose))
            self.amr.work = []
            sleep(hold_time)

        elif debug == True:
            print("[AMR - Debug mode] AMR Start Moving ... (target pose = {})".format(target_pose))
            sleep(1.0)
            print("[AMR - Real mode] Arrived at [{}] !!!".format(target_pose))
            sleep(hold_time)


    def makeRobotTaskQueue(self, printer_id='printer1', task_type='specimen_task'):
        if task_type == 'specimen_task':
            printer_number = int(printer_id.split('printer')[1])
            sensor_number = printer_number  ## 임시 (로봇쪽 sensor number 작업 필요)

            task_get_bed = [ACTION_HOME, ACTION_TOOLCHANGE_1_ATTACH, TASK_3DP_BED_OUT - printer_number, ACTION_HOME]
            task_detach_specimen = [ACTION_TOOLCHANGE_1_DETACH, ACTION_TOOLCHANGE_2_ATTACH, ACTION_HOME, TASK_DETACH_SPECIMEN]
            task_specimen_to_rack = [TASK_SPECIMEN_TO_RACK, TASK_RACK_ALIGN, ACTION_HOME]
            task_return_bed = [ACTION_TOOLCHANGE_2_DETACH, ACTION_TOOLCHANGE_1_ATTACH, TASK_3DP_BED_IN + printer_number, ACTION_HOME, ACTION_TOOLCHANGE_1_DETACH]
            self.cobot_recent_work = ACTION_TOOLCHANGE_1_DETACH
            robot_task_queue = task_get_bed + task_detach_specimen + task_specimen_to_rack + task_return_bed
            return robot_task_queue

        elif task_type == 'feed_specimen':
            task_feed_specimen = [ACTION_TOOLCHANGE_2_ATTACH, ACTION_HOME, TASK_SPECIMEN_FROM_RACK, ACTION_HOME, TASK_INSTRON_SEARCH]
            self.cobot_recent_work = TASK_INSTRON_SEARCH
            robot_task_queue = task_feed_specimen
            return robot_task_queue

        elif task_type == 'watch_specimen':
            task_watch_specimen = [TASK_INSTRON_MOVEOUT]
            self.cobot_recent_work = TASK_INSTRON_MOVEOUT
            robot_task_queue = task_watch_specimen
            return robot_task_queue

        elif task_type == 'finish':
            task_finish_experiment = [ACTION_HOME, ACTION_TOOLCHANGE_2_DETACH, ACTION_HOME]
            self.cobot_recent_work = ACTION_HOME
            robot_task_queue = task_finish_experiment
            return robot_task_queue


    def executeCobot(self, robot_task_queue, wait_until_end=False, debug=False):
        if debug == False:
            while len(robot_task_queue) != 0:
                if self.device_dict['R_001/cobot'].getStatus()['status'] == 'Standby':
                    next_task = robot_task_queue.pop(0)
                    self.device_dict['R_001/cobot'].sendCommand({'command': next_task})
                    print("[Cobot - Real mode] Robot task queue: {}".format(robot_task_queue))
                    sleep(3.0)
        elif debug == True:
            while len(robot_task_queue) != 0:
                next_task = robot_task_queue.pop(0)
                print("[Cobot - Debug mode] Robot task queue: {}".format(robot_task_queue))
                sleep(0.5)
        if wait_until_end == True: self.waitDeviceStatus(device_name='R_001/cobot', status_key='recent_work', status_value=self.cobot_recent_work)
        print("[Cobot] Robot task finished !!! (Queue is empty)")


    '''#####################################################################################################
        Instron related
    '''
    def executeInstron(self, subject_name, command_type, debug=False):
        if debug == False:
            if command_type == 'setup':
                self.waitDeviceStatus(device_name='instron', status_value='Idle')
                self.device_dict['instron'].sendCommand({'setup': subject_name})
                print("[Instron - Real mode] Test Initializing ({}) ...".format(subject_name))
            elif command_type == 'execute':
                self.waitDeviceStatus(device_name='instron', status_value='Ready')
                self.device_dict['instron'].sendCommand({'execute': subject_name})
                print("[Instron - Real mode] Test Start ({}) !!!".format(subject_name))
        
        elif debug == True:
            if command_type == 'setup':
                print("[Instron - Debug mode] Test Initializing ({}) ...".format(subject_name))
                sleep(1.0)
            elif command_type == 'execute':
                print("[Instron - Debug mode] Test Start ({}) !!!".format(subject_name))
                sleep(1.0)

        # if command_type == 'setup':
        #     if self.instron_save_flag == True:
        #         self.waitDeviceStatus(device_name='instron', status_value='data_sent')
        #         self.device_dict['instron'].sendCommand({'setup': subject_name})
        #         self.instron_save_flag = False
        #     else:
        #         self.waitDeviceStatus(device_name='instron', status_value='Idle')
        #         self.device_dict['instron'].sendCommand({'setup': subject_name})

        # elif command_type == 'execute':
        #     self.waitDeviceStatus(device_name='instron', status_value='Ready')
        #     self.device_dict['instron'].sendCommand({'execute': subject_name})
        #     print("[Instron] Test Start ({}) !!!".format(subject_name))

        # elif command_type == 'result':
        #     self.waitDeviceStatus(device_name='instron', status_value='Idle')
        #     self.device_dict['instron'].sendCommand({'result': subject_name})
        #     print("[Instron] Result file saving ... ({}) !!!".format(subject_name))
        #     self.instron_save_flag = True
            
        
    '''#####################################################################################################
        Execution manager related
    ''' 
    def checkExecutionMode(self):
        while True:
            if self.bool_auto_mode == True:
                break
            elif self.step_flag == True:
                self.step_flag = False
                break


    def waitDeviceStatus(self, device_name, status_key='status', status_value=''):
        while True:
            try:
                status_int = int(status_value)
            except:
                status_int = status_value
            # print("[DEBUG] Waiting for '{}' status to be '{}'... ".format(device_name, status))
            if self.device_dict[device_name].getStatus()[status_key] == str(status_value) or self.device_dict[device_name].getStatus()[status_key] == status_int:
                # print("[DEBUG] '{}' status = '{}' !!! ".format(device_name, status))
                break


    def executionManager(self):
        step = 0 #; printer_id = 'printer2'; subject_id = 'test2'
        debug = True
        
        while True:
            try:
                if step == 0: ## Step 0. 시편 제작 완료 시 시작
                    self.checkExecutionMode()
                    print("[Execution Manager] Step 1. Fabricating specimens for experiment... (finnished: {})".format(self.printer_list_finished))
                    printer_id = self.printer_list_finished.pop(0)
                    subject_id = self.device_info[printer_id]['subject_name']

                    printer_number = int(printer_id.split('printer')[1])
                    amr_pos_3dp = copy.deepcopy(AMR_POS_3DP_0);   amr_pos_3dp[1] += printer_number * AMR_OFFSET_3DP
                    step = 1

                if step == 1: ## Step 2-1. Get printing bed from printer (3DP bed: 3D printer -> Robot)
                    self.checkExecutionMode()
                    print("[Execution Manager] Step 2-1 (1 of 2). AMR moving... (target: {}: {})".format(printer_id, amr_pos_3dp))
                    self.executeAMR(spot_name=printer_id, target_pose=amr_pos_3dp, hold_time=0.0, debug=debug)

                    print("[Execution Manager] Step 2-1 (2 of 2). Robot task start !!! (3DP bed: {} -> Robot)".format(printer_id))
                    robot_task_queue = self.makeRobotTaskQueue(printer_id, task_type='bed_from_printer_to_robot')
                    self.executeCobot(robot_task_queue, debug=debug)
                    step = 2
                
                if step == 2: ## Step 2-2. Measurement (3DP bed: Robot -> OMM -> Robot)
                    self.checkExecutionMode()
                    print("[Execution Manager] Step 2-2 (1 of 4). AMR moving... (target: OMM, {})".format(AMR_POS_OMM))
                    self.executeAMR(spot_name='omm', target_pose=AMR_POS_OMM, hold_time=0.0, debug=debug)
                    
                    print("[Execution Manager] Step 2-2 (2 of 4). Robot task start !!! (3DP bed: Robot -> OMM)")
                    robot_task_queue = self.makeRobotTaskQueue(task_type='bed_from_robot_to_omm') # TODO: Robot task queue 짜야함 (bed_from_robot_to_omm)
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)

                    print("[Execution Manager] Step 2-2 (3 of 4). Measuring a dimension of specimen ({})".format(subject_id))
                    # TODO: OMM 신호 보내는 코드 삽입

                    print("[Execution Manager] Step 2-2 (4 of 4). Robot task start !!! (3DP bed: OMM -> Robot)")
                    robot_task_queue = self.makeRobotTaskQueue(task_type='bed_from_omm_to_robot')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)
                    step = 3
                
                if step == 3: ## Step 2-3. Get ready for the next print (시편 분리 후 3DP bed: Robot -> 3DP)
                    self.checkExecutionMode()
                    print("[Execution Manager] Step 2-3 (1 of 3). Robot task start !!! (specimen handling: {})".format(subject_id))
                    robot_task_queue = self.makeRobotTaskQueue(task_type='specimen_handling') # TODO: Robot task queue 짜야함 (specimen_handling: 시편 분리, Rack 저장)
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)

                    print("[Execution Manager] Step 2-3 (2 of 3). AMR moving... (target: {}, {})".format(printer_id, amr_pos_3dp))
                    self.executeAMR(spot_name=printer_id, target_pose=amr_pos_3dp, hold_time=0.0, debug=debug)
                                        
                    print("[Execution Manager] Step 2-3 (3 of 3). Robot task start !!! (3DP bed: Robot -> {})".format(printer_id))
                    robot_task_queue = self.makeRobotTaskQueue(printer_id, task_type='bed_from_robot_to_printer')
                    self.executeCobot(robot_task_queue, debug=debug)
                    self.printer_list_robot_done.append(printer_id)
                    step = 4

                if step == 4: ## Step 3-1. 협동로봇 시편 -> 인장시험기 (printer_id)
                    self.checkExecutionMode()
                    print("[Execution Manager] Step 3-1 (1 of 2). AMR moving... (target: instron, {})".format(AMR_POS_INSTRON))
                    self.executeAMR(spot_name='instron', target_pose=AMR_POS_INSTRON, hold_time=0.0, debug=debug)

                    print("[Execution Manager] Step 3-1 (2 of 2). Robot task start !!! (Feeding specimen: {})".format(subject_id))
                    robot_task_queue = self.makeRobotTaskQueue(task_type='feed_specimen')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)
                    step = 5
                
                if step == 5: ## Step 3-2. 인장시험 준비 (subject_id)
                    self.checkExecutionMode()
                    print("[Execution Manager] Step 3-2 (1 of 2). Experiment initializing ... (subject: {})".format(subject_id))
                    self.executeInstron(subject_id, command_type='setup');   sleep(10.0)

                    print("[Execution Manager] Step 3-2 (2 of 2). Robot task start !!! (Monitor experiment: {})".format(subject_id))
                    robot_task_queue = self.makeRobotTaskQueue(task_type='watch_specimen')
                    self.executeCobot(robot_task_queue, debug=debug)
                    step = 6
                
                if step == 6: ## Step 3-3. 인장시험 실행 & 저장 (subject_id)
                    self.checkExecutionMode()
                    print("[Execution Manager] Step 3-3 (1 of 2). Experiment start !!! (subject: {})".format(subject_id))
                    self.executeInstron(subject_id, command_type='execute')

                    print("[Execution Manager] Step 3-3 (1 of 2). Robot task start !!! (Finnishing experiment: {})".format(subject_id))
                    robot_task_queue = self.makeRobotTaskQueue(task_type='finish')
                    self.executeCobot(robot_task_queue, debug=debug)
                    step = 7 if len(self.printer_list_finished) == 0 else 0
                
                if step == 7: ## Step 4. AMR 이동 (home)
                    self.checkExecutionMode()
                    print("[Execution Manager] Step 4. AMR moving... (target: home, {})".format(AMR_POS_ZERO))
                    if debug == False: self.executeAMR(spot_name='home', target_pose=AMR_POS_ZERO, hold_time=0.0) # target_pose 수정 작업 필요
                    step = 0

            except:
                print("[Execution Manager] ERROR !!!")
                pass

            sleep(3.0)

    




if __name__ == '__main__':
    SERVER_IP = '192.168.0.81'
    # SERVER_IP = '192.168.60.101'
    test_setting = {'header_id': 'wonjae',
                    'experiment_type': 'Tensile Test',
                    'factors': [ 
                                 {'factor_name': 'infill_line_distance', 'factor_range': [1, 6]},
                                 {'factor_name': 'layer_height', 'factor_range': [0.1, 0.2]},
                                #  {'factor_name': 'default_material_print_temperature', 'factor_range': [190, 220]}, 
                               ],
                    'doe_type': DOE_GENERALIZED_FACTORIAL, # DOE_GENERALIZED_FACTORIAL=3
                    'option': [
                                [1, 2, 4, 6], 
                                [0.1, 0.12, 0.2], 
                                # [190, 191, 219, 220]
                              ],
                    }

    test_manager = TestManager(test_setting=test_setting, ip=SERVER_IP)
    test_id_list = test_manager.getTestIDs()

    rospy.init_node('DeviceManager')

    manager = DeviceManager()
    # manager.addDevice('printer1', DeviceClass_3DP(device_name='printer1', port_='5000'))

    # manager.addDevice('R_001/cobot', device_class=None)
    # manager.addDevice('instron')
    manager.addPrintingQueue(test_id_list)

    manager.addDevice('printer1', DeviceClass_3DP(device_name='printer1', ip_=SERVER_IP, port_='5001', usb_port_=0))
    # manager.addDevice('printer2', DeviceClass_3DP(device_name='printer2', ip_=SERVER_IP, port_='5002', usb_port_=1))
    # manager.addDevice('printer3', DeviceClass_3DP(device_name='printer3', ip_=SERVER_IP, port_='5003', usb_port_=2))
    # manager.addDevice('printer4', DeviceClass_3DP(device_name='printer4', ip_=SERVER_IP, port_='5004', usb_port_=3))
    sleep(3.0)
    manager.device_dict['printer1'].sendCommand({"connection": True})
    # manager.device_dict['printer2'].sendCommand({"connection": True})
    
    # manager.device_dict['printer3'].sendCommand({"connection": True})



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