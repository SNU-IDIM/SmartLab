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
            smartlab_cmd['test_step'] = 1

            
            print('gi')

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
