#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, time, json, zmq
import numpy as np

class SmartLabClient(object):

    def __init__(self, ip='192.168.60.21'):
        try:
            self.context = zmq.Context()
            print("[DEBUG - ZMQ] Connecting...")
            self.socket = self.context.socket(zmq.REQ) # REQuest
            self.socket.connect("tcp://{}:5555".format(ip)) # Change LocalHost IP 
            print("[DEBUG - ZMQ] Client initialized!")
        except:
            print("[ERROR - ZMQ] Client setting error")
        time.sleep(3.0)

    
    def send(self, command):
        self.socket.send_string(json.dumps(command))
        response = self.socket.recv()
        return response
        


if __name__ == '__main__':
    command = dict()
    command['test_mode'] = 'auto'
    command['test_step'] = 0
    command['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1', 'printer2', 'printer3']
    command['setup_doe'] = {
                            'header_id': 'DRY_TEST',
                            'experiment_type': 'Tensile Test',
                            'factors': [ {'factor_name': 'infill_line_distance', 'factor_range': [0.4, 0.45]},
                                        {'factor_name': 'infill_angles'}
                                    ],
                            'doe_type': 3, # DOE_GENERALIZED_FACTORIAL=3
                            'option': [ [0.4, 0.45], 
                                        ['0', '45,135', '0,90', '90']
                                    ],
                            }

    smartlab = SmartLabClient()
    smartlab.send(command)
