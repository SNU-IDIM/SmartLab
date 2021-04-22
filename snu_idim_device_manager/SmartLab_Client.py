#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, time, json, zmq
import numpy as np

class SmartLabClient(object):

    def __init__(self, ip='192.168.60.21'):
        
        ## Subject Info. definition
        try:
            self.context = zmq.Context()
            print("[DEBUG - ZMQ] Connecting...")
            self.socket = self.context.socket(zmq.REQ) # REQuest
            self.socket.connect("tcp://{}:5555".format(ip)) # Change LocalHost IP 
            print("[DEBUG - ZMQ] Client initialized!")
        except:
            print("[ERROR - ZMQ] Client setting error")

        time.sleep(3.0)

        try:
            smartlab_cmd = dict()
            smartlab_cmd['test_mode'] = 'auto'
            smartlab_cmd['test_step'] = 0
            smartlab_cmd['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1', 'printer2', 'printer3']
            smartlab_cmd['setup_doe'] = {
                                    'header_id': 'DRY_TEST',
                                    'experiment_type': 'Tensile Test',
                                    'factors': [ 
                                                {'factor_name': 'infill_line_distance', 'factor_range': [1, 6]},
                                                {'factor_name': 'layer_height', 'factor_range': [0.1, 0.2]},
                                            ],
                                    'doe_type': 3,
                                    'option': [
                                                [1, 2, 4, 6], 
                                                [0.1, 0.12, 0.2], 
                                            ],
                                    }
            # print(smartlab_cmd)
            

            while True:
                self.socket.send_string(json.dumps(smartlab_cmd))
                response = json.loads(self.socket.recv())
                time.sleep(3)

                print("============")
                device_list = list()
                for i in response['device']:
                    device_list.append(i)
                print("Device list: {}".format(device_list))
                for i in response['experiment']:
                    print("{}: {}".format(i, response['experiment'][i]))
   
                    # print(response[i])
            # print("[DEBUG - ZMQ] Response from server: {}".format(response))

        except:
            print('[ERROR - ZMQ] Something is wrong ...')


test = SmartLabClient()
