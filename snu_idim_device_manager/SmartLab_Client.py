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
            smartlab_cmd['test_mode'] = 'step'
            smartlab_cmd['test_step'] = -1
            smartlab_cmd['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1', 'printer2', 'printer3']
            smartlab_cmd['setup_doe'] = {
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
            

            for i in range(10):
                print(i)
                self.socket.send_string(json.dumps(smartlab_cmd))
                response = json.loads(self.socket.recv())
                time.sleep(30)
                for i in response:
                    print(response[i])
            # print("[DEBUG - ZMQ] Response from server: {}".format(response))

        except:
            print('[ERROR - ZMQ] Something is wrong ...')


test = SmartLabClient()
