#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, json, time
import datetime
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

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from SqlHelper import SqlHelper
from Cam_Streaming_Server import Cam_Streaming_Server

from DevicePluginToROS import DevicePluginToROS

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_amr")) )
from DeviceClass_AMR import DeviceClass_AMR

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_3dp")) )
from DeviceClass_3DP import DeviceClass_3DP

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_omm")) )
from DeviceClass_OMM import DeviceClass_OMM


# sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_instron")) )
# from DeviceClass_Instron import DeviceClass_Instron


class TestManager():
    def __init__(self, test_setting, ip='localhost'):
        self.test_designer = TestDesigner(test_setting=test_setting, ip=ip)
        self.test_id_list = self.test_designer.sendDOE()
        self.testsets = self.test_designer.testsets
    
    def getTestIDs(self):
        print("[DEBUG - TestManager] Test ID list: \n{}".format(self.test_id_list))
        return self.test_id_list



class SmartLABCore():
    def __init__(self, ip_='192.168.60.21', port_=5555):
        ## Initializating SmartLAB
        self.ip_ = ip_
        self.port_ = port_

        self.init_server_flag = True
        self.init_db_flag = True
        self.test_step = 0

        ## Connect to the database server
        self.mysql = SqlHelper(host='localhost', username='root', password='0000', port=3306, database='SmartLab')

        ## Check existing columns and data
        column_list = self.mysql.get_table_columns(tablename='result')
        print("[DEBUG] Column list: \n{} in table ({}): \n".format(column_list, 'result'))

        self.req = dict()
        self.req['test_mode'] = 'step'
        self.req['test_step'] = -1
        self.req['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer3']
        self.req['setup_doe'] = {
                                'header_id': 'DRY_TEST',
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
        self.res = dict()

        ## Device dict (list of devices to handle with ID)
        self.device_dict = dict() # for device manager
        self.device_info = dict() # to send data to client via ZMQ

        ## Test info
        self.test_info = dict()
        self.test_info['waiting'] = list()
        self.test_info['fabrication'] = list()
        self.test_info['measurement'] = list()
        self.test_info['experiment'] = list()
        self.test_info['completed'] = list()

        ## Camera Streaming
        self.streaming_server = Cam_Streaming_Server(cam_list=['overview', 'cobot_eef', 'cobot_front'])

        ## ZMQ: ROS(server) <-> Python(client)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("tcp://*:{}".format(self.port_))

        ## ZMQ Server thread
        self.thread_server = Thread(target=self.zmq_server)
        self.thread_server.start()


    def __del__(self):
        self.thread_server.terminate()
        self.thread_1.terminate()
        self.thread_2.terminate()
        self.thread_3.terminate()
        pass




    '''#####################################################################################################
        Interface of SmartLab Server
    '''
    def zmq_server(self):
        while True:
            self.req.update(json.loads(self.socket.recv()))
            if self.req['setup_doe']['doe_type'] == 'GENERALIZED_FACTORIAL':
                self.req['setup_doe']['doe_type'] = DOE_GENERALIZED_FACTORIAL
            # print('[DEBUG] Request from SmartLab Client: {}'.format(self.req))

            ## Initializing ...
            if self.init_server_flag == True:
                sleep(3.0)
                self.init_server_flag = False

                test_manager = TestManager(test_setting=self.req['setup_doe'], ip=self.ip_)
                self.connectDevices(self.req['setup_device'])
                test_id_list = test_manager.getTestIDs()
                self.addPrintingQueue(test_id_list)
                print(type(test_manager.testsets))
                print(test_manager.testsets)
                sleep(3.0)

                doe_dict = test_manager.testsets.to_dict()
                try:
                    for test_id in self.mysql.select('result', 'subject_name', conds=["subject_name like '%{}%'".format(self.req['setup_doe']['header_id'])]):
                        self.mysql.delete('result', {'subject_name': test_id})
                except:
                    pass

                for factor in doe_dict:
                    try:
                        list(self.mysql.get_table_columns(tablename='result')).index(factor)
                        print("[DEBUG] Column already exists !!! ({})".format(factor))
                    except:
                        self.mysql.add_column('result', factor, 'varchar(64)')
                        print("[DEBUG] Column not exists !!! ({})".format(factor))
                    
                    if factor != 'NUMBER':
                        for test_id in test_id_list:
                            n_id = int(test_id.split('_')[-1])
                            print(n_id)
                            print(doe_dict)
                            print(doe_dict[factor])
                            print(doe_dict[factor][n_id])
                            # self.mysql.insert('result', {'subject_name': test_id, 'Status': 'Waiting'}, conds='ON DUPLICATE KEY UPDATE Status = "-"')
                            self.mysql.insert('result', {'subject_name': test_id, 'Status': 'Waiting', str(factor): str(doe_dict[factor][n_id])}, conds='ON DUPLICATE KEY UPDATE {} = \'{}\''.format(str(factor), str(doe_dict[factor][n_id])))
                            
                    
                    # for test_id in doe_dict[factor]:
                    #     subject_name = "{}_{}".format(self.req['setup_doe']['header_id'], test_id)
                    #     print("---")
                    #     print(subject_name)
                    #     self.mysql.insert('result', {'subject_name': subject_name, 'Status': 'Waiting'}, conds='WHERE subject_name = \'{}\''.format(subject_name))


                # for test_id in test_id_list:
                #     # self.mysql.insert('result', {'subject_name': test_id, 'Status': 'Waiting'}, conds='ON DUPLICATE KEY UPDATE Status = "-"')
                #     for factor in doe_dict:
                #         self.mysql.insert('result', {factor, doe_dict[factor][test_id.split('_')[-1]]}, conds='WHERE subject_name = \'{}\''.format(test_id))


                # for test_id in test_id_list:
                #     try:
                #         self.mysql.delete('result', {'subject_name': test_id})
                #     except:
                #         print("[WARN] MYSQL error while deleting data from table 'result'")
                #     self.mysql.insert('result', {'subject_name': test_id, 'Status': 'Waiting'}, conds='ON DUPLICATE KEY UPDATE Status = "-"')
                                
                ## 3DP Manager thread
                self.thread_1 = Thread(target=self.manager3DP)
                self.thread_1.start()

                ## Cobot Manager thread
                self.thread_2 = Thread(target=self.executionManager)
                self.thread_2.start()

                ## Refreshing 'device_info' thread (for Client)
                self.thread_3 = Thread(target=self.refreshDeviceInfo)
                self.thread_3.start()

            # self.socket.send_string('{}, {}, {}'.format(self.req['test_mode'], self.req['test_step'], self.test_step))

            try:
                self.res['device'] = self.device_info
                self.res['experiment'] = self.test_info
                # self.socket.send_string(json.dumps(self.device_info))
                self.socket.send_string('{}, {}, {}'.format(self.req['test_mode'], self.req['test_step'], self.test_step))
            except:
                print("[ERRRRRRRRRRRRr]")
                # self.socket.send_string('{}, {}, {}'.format(self.req['test_mode'], self.req['test_step'], self.test_step))



    '''#####################################################################################################
        Device manager related
    '''
    def connectDevices(self, device_list):
        print('[DEBUG] Trying to connect devices ... ({})'.format(device_list))

        for device_id in device_list:
            if device_id == 'R_001/amr':
                self.addDevice(device_id, device_class=DeviceClass_AMR(device_name=device_id))
                print('[DEBUG] Device connected !!! ({})'.format(device_id))

            elif device_id.find('printer') != -1:
                socket_port = '500{}'.format(device_id.split('printer')[1]) # socket port: '5001', '5002', '5003', '5004'
                usb_port = int(device_id.split('printer')[1]) - 1 # usb port: 0, 1, 2, 3
                self.addDevice(device_id, DeviceClass_3DP(device_name=device_id, ip_=self.ip_, port_=socket_port, usb_port_=usb_port))
                print('[DEBUG] Device connected !!! ({})'.format(device_id))

            elif device_id == 'R_001/cobot' or device_id == 'instron' or device_id == 'MS':
                self.addDevice(device_id)
                print('[DEBUG] Device connected !!! ({})'.format(device_id))
            

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
            self.device_info = dict()
            keys = self.device_dict.keys()
            values = self.device_dict.values()

            for i in range(len(keys)):
                self.device_info[keys[i]] = self.device_dict[keys[i]].getStatus()
            try:
                mysql = SqlHelper(host='localhost', username='root', password='0000', port=3306, database='SmartLab')
                device_data = dict()
                for device_id in self.device_info:
                    device_id_db = str(device_id.split('/')[1]) if device_id.find('/') != -1 else str(device_id)
                    device_data[device_id_db] = json.dumps(self.device_info[device_id])
                mysql.insert('device_info', device_data)
            except:
                print("[ERROR] Device information refresh failed !!!")

            sleep(1.0)


    '''#####################################################################################################
        3D Printer related
    '''
    def addPrintingQueue(self, printing_queue):
        self.printing_queue            = printing_queue
        self.printer_list_idle         = list()
        self.printer_list_initializing = list()
        self.printer_list_printing     = list()
        self.printer_list_finished     = list()  ## ***
        self.printer_list_robot_done   = list()

        self.specimen_ready_list = list()  ## ***
        self.test_ready_list     = list()  ## ***


    def tapping(self, printer_name, gCode_name):
        gCode_name = gCode_name + '.gcode'
        UPLOAD_PATH = os.path.join(os.getenv("HOME"), '.octoprint/uploads/')
        GCODE_PATH = os.path.join(UPLOAD_PATH, self.req['setup_doe']['header_id'], gCode_name)
        CODE_PATH = os.path.join(os.getenv("HOME"), 'catkin_ws/src/SNU_SmartLAB', 'snu_idim_slicer','Tapping.py')
        command = 'python3 ' + CODE_PATH + ' ' + GCODE_PATH + ' ' + printer_name
        print(command)
        os.system(command)


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

                        if device_status['status'].find('Printing') != -1:
                            try:
                                self.printer_list_printing.index(device_id)
                            except:
                                self.printer_list_printing.append(device_id)

                        ## Check whether printing job is done
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
                                if self.req['setup_doe']['header_id'] != 'DRY_TEST':
                                    self.tapping(device_id, print_next)
                                self.device_dict[device_id].sendCommand({'print': print_next})
                                self.mysql.insert('result', {'subject_name': print_next}, conds='ON DUPLICATE KEY UPDATE Status = "Fabrication"')
                                sleep(2.0)
                                print("[3DP] {} status: Idle -> Printing {}".format(device_id, print_next))
                            except:
                                print("[3DP] Printing queue is empty !!!")
                except:
                    pass
                    
            sleep(3.0)



    '''#####################################################################################################
        Robot related
    '''
    def executeAMR(self, target_pose, spot_name="default", wait_until_end=True, hold_time=0.0, debug=False):
        cmd_dict = {'spot_name': spot_name, 
                    'target_pose': target_pose, 
                    'hold_time': hold_time, }
        if debug == False:
            print("[AMR - Real mode] AMR Start Moving ... (target pose = {})".format(target_pose))
            self.device_dict['R_001/amr'].sendCommand(cmd_dict)
            print("[AMR - Real mode] Arrived at [{}] !!!".format(cmd_dict['target_pose']))

        elif debug == True:
            print("[AMR - Debug mode] AMR Start Moving ... (target pose = {})".format(target_pose))
            wait_until_end = False
            sleep(1.0)
            print("[AMR - Debug mode] Arrived at [{}] !!!".format(target_pose))
            sleep(hold_time)

        if wait_until_end == True: self.waitDeviceStatus(device_name='R_001/amr', status_key='status', status_value='Idle')


    def makeRobotTaskQueue(self, printer_id='printer0', task_type='specimen_task'):
        if task_type == 'bed_from_printer_to_robot':
            printer_number = int(printer_id.split('printer')[1])
            task_tool_change_0_to_1 = [ACTION_HOME, ACTION_TOOLCHANGE_1_ATTACH, ACTION_HOME]
            task_get_bed = [ACTION_HOME, TASK_3DP_BED_OUT - printer_number, ACTION_HOME]
            self.cobot_recent_work = ACTION_HOME
            robot_task_queue = task_tool_change_0_to_1 + task_get_bed
            return robot_task_queue
        
        elif task_type == 'bed_from_robot_to_omm':
            task_place_bed = [ACTION_HOME, TASK_3DP_BED_IN] # TODO: 로봇 작업 추가 필요
            self.cobot_recent_work = TASK_3DP_BED_IN
            robot_task_queue = task_place_bed
            return robot_task_queue
        
        elif task_type == 'bed_from_omm_to_robot':
            task_get_bed = [ACTION_HOME, TASK_3DP_BED_OUT, ACTION_HOME] # TODO: 로봇 작업 추가 필요
            self.cobot_recent_work = ACTION_HOME
            robot_task_queue = task_get_bed
            return robot_task_queue

        elif task_type == 'specimen_handling':
            task_tool_change_1_to_2 = [ACTION_HOME, ACTION_TOOLCHANGE_1_DETACH, ACTION_TOOLCHANGE_2_ATTACH, ACTION_HOME]
            task_detach_specimen = [ACTION_HOME, TASK_DETACH_SPECIMEN, ACTION_HOME]
            task_specimen_to_rack = [ACTION_HOME, TASK_SPECIMEN_TO_RACK, TASK_RACK_ALIGN, ACTION_HOME]
            self.cobot_recent_work = ACTION_HOME
            robot_task_queue = task_tool_change_1_to_2 + task_detach_specimen + task_specimen_to_rack
            return robot_task_queue
        
        elif task_type == 'bed_from_robot_to_printer':
            printer_number = int(printer_id.split('printer')[1])
            task_tool_change_2_to_1 = [ACTION_HOME, ACTION_TOOLCHANGE_2_DETACH, ACTION_TOOLCHANGE_1_ATTACH, ACTION_HOME]
            task_return_bed = [ACTION_HOME, TASK_3DP_BED_IN + printer_number, ACTION_HOME]
            self.cobot_recent_work = ACTION_HOME
            robot_task_queue = task_tool_change_2_to_1 + task_return_bed
            return robot_task_queue

        elif task_type == 'feed_specimen':
            task_tool_change_1_to_2 = [ACTION_HOME, ACTION_TOOLCHANGE_1_DETACH, ACTION_TOOLCHANGE_2_ATTACH, ACTION_HOME]
            task_feed_specimen = [ACTION_HOME, TASK_SPECIMEN_FROM_RACK, ACTION_HOME, TASK_INSTRON_SEARCH]
            self.cobot_recent_work = TASK_INSTRON_SEARCH
            robot_task_queue = task_tool_change_1_to_2 + task_feed_specimen
            return robot_task_queue

        elif task_type == 'monitor_experiment':
            task_monitor_experiment = [TASK_INSTRON_MOVEOUT]
            self.cobot_recent_work = TASK_INSTRON_MOVEOUT
            robot_task_queue = task_monitor_experiment
            return robot_task_queue

        elif task_type == 'remove_specimen1':
            task_remove_specimen1 = [TASK_INSTRON_CLEAN1]
            self.cobot_recent_work = TASK_INSTRON_CLEAN1
            robot_task_queue = task_remove_specimen1
            return robot_task_queue
        
        elif task_type == 'remove_specimen2':
            task_remove_specimen2 = [TASK_INSTRON_CLEAN2]
            self.cobot_recent_work = TASK_INSTRON_CLEAN2
            robot_task_queue = task_remove_specimen2
            return robot_task_queue
        
        elif task_type == 'finish_experiment':
            task_tool_change_2_to_0 = [ACTION_HOME, ACTION_TOOLCHANGE_2_DETACH, ACTION_HOME]
            self.cobot_recent_work = ACTION_HOME
            robot_task_queue = task_tool_change_2_to_0
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
            wait_until_end = False
            while len(robot_task_queue) != 0:
                next_task = robot_task_queue.pop(0)
                print("[Cobot - Debug mode] Robot task queue: {}".format(robot_task_queue))
                sleep(0.5)
        if wait_until_end == True: self.waitDeviceStatus(device_name='R_001/cobot', status_key='recent_work', status_value=self.cobot_recent_work)
        print("[Cobot] Robot task finished !!! (Queue is empty)")



    '''#####################################################################################################
        On-Machine Measurment (OMM) related
    '''
    def executeOMM(self, subject_name, command_type, wait_until_end=False, debug=False):
        if debug == False:
            sleep(2.0)
            self.waitDeviceStatus(device_name='MS', status_value='Idle')
            if command_type == 'measure_dimension':
                print("[OMM - Real mode] Measuring dimension ({}) ...".format(subject_name))
                self.device_dict['MS'].sendCommand({command_type: subject_name})
            sleep(10.0)
            self.waitDeviceStatus(device_name='MS', status_value='Idle')

        elif debug == True:
            if command_type == 'measure_dimension':
                print("[OMM - Debug mode] Measuring dimension ({}) ...".format(subject_name))
                sleep(1.0)



    '''#####################################################################################################
        Instron related
    '''
    def executeInstron(self, subject_name, command_type, wait_until_end=False, debug=False):
        if debug == False:
            if command_type == 'setup':
                self.waitDeviceStatus(device_name='instron', status_value='Idle')
                self.device_dict['instron'].sendCommand({command_type: subject_name})
                print("[Instron - Real mode] Test Initializing ({}) ...".format(subject_name))
                if wait_until_end == True: self.waitDeviceStatus(device_name='instron', status_key='status', status_value='Ready')
            elif command_type == 'execute':
                self.waitDeviceStatus(device_name='instron', status_value='Ready')
                self.device_dict['instron'].sendCommand({command_type: subject_name})
                print("[Instron - Real mode] Test Start ({}) !!!".format(subject_name))
                if wait_until_end == True: self.waitDeviceStatus(device_name='instron', status_key='status', status_value='Done')
            elif command_type == 'open':
                self.waitDeviceStatus(device_name='instron', status_value='Done')
                self.device_dict['instron'].sendCommand({command_type: subject_name})
                sleep(10.0)
                print("[Instron - Real mode] Analizing Start ({}) !!!".format(subject_name))
                if wait_until_end == True: self.waitDeviceStatus(device_name='instron', status_key='status', status_value='Idle')
        
        elif debug == True:
            if command_type == 'setup':
                print("[Instron - Debug mode] Test Initializing ({}) ...".format(subject_name))
                sleep(1.0)
            elif command_type == 'execute':
                print("[Instron - Debug mode] Test Start ({}) !!!".format(subject_name))
                sleep(1.0)



    '''#####################################################################################################
        Execution manager related
    ''' 
    def checkExecutionMode(self):
        while True:
            mysql = SqlHelper(host='localhost', username='root', password='0000', port=3306, database='SmartLab')
            mysql.insert('system_status', {'id': '1', 'control_mode': str(self.req['test_mode'])}, conds='ON DUPLICATE KEY UPDATE {} = \'{}\''.format('control_mode', self.req['test_mode']))
            mysql.insert('system_status', {'id': '1', 'control_step': str(self.test_step)}, conds='ON DUPLICATE KEY UPDATE {} = \'{}\''.format('control_step', self.test_step))
            mysql.insert('system_status', {'id': '1', 'control_status': 'Waiting'}, conds='ON DUPLICATE KEY UPDATE {} = \'{}\''.format('control_status', 'Waiting'))
            if self.req['test_mode'] == 'auto':
                print("[AUTO MODE] Execute next step automatically ... (Step: {})".format(self.test_step))
                mysql.insert('system_status', {'id': '1', 'control_status': 'Running'}, conds='ON DUPLICATE KEY UPDATE {} = \'{}\''.format('control_status', 'Running'))
                return 0
            elif self.req['test_mode'] == 'step':
                print("[STEP MODE] Waiting for triggering ... (Step: {})".format(self.test_step))
                
                if self.req['test_step'] != -1:
                    print("[STEP MODE] Execute next step ... (Step: {})".format(self.test_step))
                    mysql.insert('system_status', {'id': '1', 'control_status': 'Running'}, conds='ON DUPLICATE KEY UPDATE {} = \'{}\''.format('control_status', 'Running'))
                    self.req['test_step'] = -1
                    return 1
            elif self.req['test_mode'] == 'debug':
                self.test_step = self.req['test_step']
                print("[DEBUG MODE] Execute following step ... (Step: {})".format(self.test_step))
                return -1


    def waitDeviceStatus(self, device_name, status_key='status', status_value=''):
        sleep(3.0)
        while True:
            try:
                status_int = int(status_value)
            except:
                status_int = status_value
            if self.device_dict[device_name].getStatus()[status_key] == str(status_value) or self.device_dict[device_name].getStatus()[status_key] == status_int:
                break


    def executionManager(self):
        debug = False
        debug_withoutAMR = False #True

        while True:
            try:
                ## Check the execution mode (auto: 0 / step: 1 / debug: -1)
                if self.checkExecutionMode() == -1:
                    printer_id = 'printer3'
                    subject_id = 'yun_9'
                    printer_number = 3
                    amr_pos_3dp = deepcopy(AMR_POS_3DP_0)
                    amr_pos_3dp[1] += printer_number * AMR_OFFSET_3DP


                if self.test_step == 0: ## Step 0. 시편 제작 완료 시 시작
                    print("[Execution Manager] Step 1. Fabricating specimens for experiment... (finnished: {})".format(self.printer_list_finished))
                    try:
                        printer_id = str(self.printer_list_finished.pop(0))
                        subject_id = str(self.device_info[printer_id]['subject_name'])
                        printer_number = int(printer_id.split('printer')[1])
                        amr_pos_3dp = deepcopy(AMR_POS_3DP_0);   amr_pos_3dp[1] += printer_number * AMR_OFFSET_3DP
                        self.test_step = 1
                    except:
                        pass


                elif self.test_step == 1: ## Step 2-1. Get printing bed from printer (3DP bed: 3D printer -> Robot)
                    self.mysql.insert('result', {'subject_name': subject_id}, conds='ON DUPLICATE KEY UPDATE Status = "Measurement"')
                    print("[Execution Manager] Step 2-1 (1 of 2). AMR moving... (target: {}: {})".format(printer_id, amr_pos_3dp))
                    if debug_withoutAMR == False:
                        self.executeAMR(spot_name=printer_id, target_pose=amr_pos_3dp, hold_time=0.0, debug=debug)
                    else:
                        print('[DEBUG_NO_AMR] AMR should be placed in front of 3DP')
                    print("[Execution Manager] Step 2-1 (2 of 2). Robot task start !!! (3DP bed: {} -> Robot)".format(printer_id))
                    robot_task_queue = self.makeRobotTaskQueue(printer_id, task_type='bed_from_printer_to_robot')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)
                    self.test_step = 2
                

                elif self.test_step == 2: ## Step 2-2. Measurement (3DP bed: Robot -> OMM -> Robot)
                    print("[Execution Manager] Step 2-2 (1 of 4). AMR moving... (target: OMM, {})".format(AMR_POS_OMM))
                    if debug_withoutAMR == False:
                        self.executeAMR(spot_name='omm', target_pose=AMR_POS_OMM, hold_time=0.0, debug=debug)
                    else:
                        print('[DEBUG_NO_AMR] AMR should be placed in front of OMM')
                        rospy.sleep(20)
                    print("[Execution Manager] Step 2-2 (2 of 4). Robot task start !!! (3DP bed: Robot -> OMM)")
                    robot_task_queue = self.makeRobotTaskQueue(task_type='bed_from_robot_to_omm')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)

                    print("[Execution Manager] Step 2-2 (3 of 4). Measuring a dimension of specimen ({})".format(subject_id))
                    self.executeOMM(subject_name=subject_id, wait_until_end=True, command_type='measure_dimension', debug=debug)

                    print("[Execution Manager] Step 2-2 (4 of 4). Robot task start !!! (3DP bed: OMM -> Robot)")
                    robot_task_queue = self.makeRobotTaskQueue(task_type='bed_from_omm_to_robot')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)
                    self.test_step = 3
                

                elif self.test_step == 3: ## Step 2-3. Get ready for the next print (시편 분리 후 3DP bed: Robot -> 3DP)
                    print("[Execution Manager] Step 2-3 (1 of 3). Robot task start !!! (specimen handling: {})".format(subject_id))
                    robot_task_queue = self.makeRobotTaskQueue(task_type='specimen_handling')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)

                    print("[Execution Manager] Step 2-3 (2 of 3). AMR moving... (target: {}, {})".format(printer_id, amr_pos_3dp))
                    if debug_withoutAMR == False:
                        self.executeAMR(spot_name=printer_id, target_pose=amr_pos_3dp, hold_time=0.0, debug=debug)
                    else:
                        print('[DEBUG_NO_AMR] AMR should be placed in front of 3DP')
                        rospy.sleep(20)

                    print("[Execution Manager] Step 2-3 (3 of 3). Robot task start !!! (3DP bed: Robot -> {})".format(printer_id))
                    robot_task_queue = self.makeRobotTaskQueue(printer_id, task_type='bed_from_robot_to_printer')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)
                    self.printer_list_robot_done.append(printer_id)
                    self.test_step = 4


                elif self.test_step == 4: ## Step 3-1. 협동로봇 시편 -> 인장시험기 (printer_id)
                    self.mysql.insert('result', {'subject_name': subject_id}, conds='ON DUPLICATE KEY UPDATE Status = "Experiment"')
                    print("[Execution Manager] Step 3-1 (1 of 2). AMR moving... (target: instron, {})".format(AMR_POS_INSTRON))
                    if debug_withoutAMR == False:
                        self.executeAMR(spot_name='instron', target_pose=AMR_POS_INSTRON, hold_time=0.0, debug=debug)
                    else:
                        print('[DEBUG_NO_AMR] AMR should be placed in front of Instron')
                        rospy.sleep(20)
                    print("[Execution Manager] Step 3-1 (2 of 2). Robot task start !!! (Feeding specimen: {})".format(subject_id))
                    robot_task_queue = self.makeRobotTaskQueue(task_type='feed_specimen')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)
                    self.test_step = 5
                

                elif self.test_step == 5: ## Step 3-2. 인장시험 준비 (subject_id)
                    print("[Execution Manager] Step 3-2 (1 of 2). Experiment initializing ... (subject: {})".format(subject_id))
                    self.executeInstron(subject_id, command_type='setup', wait_until_end=True, debug=debug)

                    print("[Execution Manager] Step 3-2 (2 of 2). Robot task start !!! (Monitor experiment: {})".format(subject_id))
                    robot_task_queue = self.makeRobotTaskQueue(task_type='monitor_experiment')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)
                    self.test_step = 6

                
                elif self.test_step == 6: ## Step 3-3. 인장시험 실행 & 저장 (subject_id)
                    print("[Execution Manager] Step 3-3 (1 of 4). Experiment start !!! (subject: {})".format(subject_id))
                    self.executeInstron(subject_id, command_type='execute', wait_until_end=True, debug=debug)

                    print("[Execution Manager] Step 3-3 (2 of 4). Removing specimen and start analyzing !!! (subject: {})".format(subject_id))
                    robot_task_queue = self.makeRobotTaskQueue(task_type='remove_specimen1') # Grasp specimen to remove
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)
                    self.executeInstron(subject_id, command_type='open', wait_until_end=False, debug=debug) # Instron gripper open
                    robot_task_queue = self.makeRobotTaskQueue(task_type='remove_specimen2') # Remove specimen
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)

                    print("[Execution Manager] Step 3-3 (4 of 4).  Robot task start !!! (Finish experiment: {})".format(subject_id))
                    robot_task_queue = self.makeRobotTaskQueue(task_type='finish_experiment')
                    self.executeCobot(robot_task_queue, wait_until_end=True, debug=debug)
                    self.mysql.insert('result', {'subject_name': subject_id}, conds='ON DUPLICATE KEY UPDATE Status = "Done"')
                    self.test_step = 7 if len(self.printer_list_finished) == 0 else 0
                

                elif self.test_step == 7: ## Step 4. 수행할 작업 없을 시 AMR -> home으로 이동
                    print("[Execution Manager] Step 4. AMR moving... (target: home, {})".format(AMR_POS_HOME))
                    if debug == False: self.executeAMR(spot_name='home', target_pose=AMR_POS_HOME, hold_time=0.0, debug=debug) # target_pose 수정 작업 필요
                    self.test_step = 0

            except:
                print("[Execution Manager] ERROR !!!")
                pass

            sleep(3.0)




if __name__ == '__main__':

    rospy.init_node('SmartLABCore')

    smartlab = SmartLABCore(ip_='192.168.60.21', port_=5555)
