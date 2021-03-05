#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, time, json
import zmq
from DOE import DOE
import numpy as np
import pyDOE2 as pyDOE
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/imp")) )
from IDIM_framework import *

class TestDesigner(object):

    def __init__(self, test_setting, ip='192.168.60.101'):
        ## Subject Info. definition
        self.test_info_dict = dict()
        self.header_ = test_setting
        self.info_ = {'experiment_sets': dict(),
                      'experiment_status': list(),}
                      
        self.typeOfExperiment = self.header_['experiment_type']
        self.factors = self.header_['factors']
        self.numberOfFactors = len(self.header_['factors'])
        self.typeOfDOE = self.header_['doe_type']

        try:
            self.context = zmq.Context()
            print("[DEBUG - ZMQ] Connecting...")
            self.socket = self.context.socket(zmq.REQ) # REQuest
            self.socket.connect("tcp://{}:6001".format(ip)) # Change LocalHost IP 
            print("[DEBUG - ZMQ] Client initialized!")
        except:
            print("[ERROR - ZMQ] Client setting error")

        self.makeDOE()
        self.sendDOE()


    def makeDOE(self):
        if self.typeOfDOE == DOE_TWO_LEVEL_FULL_FACTORIAL:
            self.output = pyDOE.ff2n(self.numberOfFactors)
            self.output = self.output.astype(str)

        elif self.typeOfDOE == DOE_THREE_LEVEL_FULL_FACTORIAL:
            self.output = pyDOE.fullfact(np.ones(
                self.numberOfFactors, dtype=int)*3) - np.ones(self.numberOfFactors, dtype=int)

        elif self.typeOfDOE == DOE_GENERALIZED_FACTORIAL:
            if self.header_['option'] is not False:
                levels = []
                for i in range(len(self.header_['option'])):
                    levels.append(len(self.header_['option'][i]))
                self.output = pyDOE.fullfact(levels)
                # print(self.output)
            else:
                print('option is not defined, option should be defined as tuple')

        elif self.typeOfDOE == DOE_TWO_LEVEL_FRACTIONAL_FACTORIAL:
            if self.header_['option'] is not False:
                self.output = pyDOE.fracfact(option)
                self.output = self.output.astype(str)
            else:
                print('option is not defined, option example : a b ab c ac bc abc')

        elif self.typeOfDOE == DOE_PLACKETT_BURMAN:
            self.output = pyDOE.pbdesign(self.numberOfFactors)

        elif self.typeOfDOE == DOE_BOX_BEHNKEN:
            self.output = pyDOE.bbdesign(self.numberOfFactors)

        elif self.typeOfDOE == DOE_CENTRAL_COMPOSITE:
            self.output = pyDOE.ccdesign(self.numberOfFactors)

        ## DOE Complete with High, Low value
        self.numberOfExperiments = len(self.output)

        ## Mapping between High, low value with actual value
        if self.typeOfDOE == DOE_GENERALIZED_FACTORIAL:
            for i in range(self.numberOfFactors):
                for j in range(self.numberOfExperiments):
                    self.output[j, i] = self.header_['option'][i][int(self.output[j, i])]

        elif self.typeOfDOE == DOE_TWO_LEVEL_FULL_FACTORIAL or self.typeOfDOE == DOE_TWO_LEVEL_FRACTIONAL_FACTORIAL:
            for i in range(self.numberOfFactors):
                lower_limit = str(factors[i][1])
                upper_limit = str(factors[i][2])
                for j in range(self.numberOfExperiments):
                    if self.output[j, i] == '-1.0':
                        self.output[j, i] = lower_limit
                    elif self.output[j, i] == '1.0':
                        self.output[j, i] = upper_limit

        else:
            for i in range(self.numberOfFactors):
                lower_limit = factors[i][1]
                upper_limit = factors[i][2]

                span = (upper_limit - lower_limit)/2
                offset = (upper_limit + lower_limit)/2
                self.output[:, i] = self.output[:, i] * span + offset

        self.output = np.concatenate((np.array([np.arange(self.numberOfExperiments)+1]).T, self.output), axis=1)

        testsets = DOE(self.typeOfExperiment, self.factors, self.output, self.typeOfDOE)
        self.testsets = testsets.df
        return self.testsets


    def getTestsetList(self):
        n = len(self.test_info_dict['doe']['NUMBER'])
        testset_list = list()
        for i in range(n):
            testset_list.append('{}_{}'.format(self.test_info_dict['header']['header_id'], i))
        return testset_list


    def sendDOE(self):
        try:
            testset_dict = dict()
            testset_dict['header'] = self.header_
            testset_dict['doe'] = self.testsets.to_dict()
            # print(testset_dict)
            self.socket.send_string(json.dumps(testset_dict))
            response = self.socket.recv()
            print("[DEBUG - ZMQ] Response from server: {}".format(response))
            self.test_info_dict = testset_dict
            
            testset_id_list = self.getTestsetList()
            return testset_id_list

        except:
            print('[ERROR - ZMQ] Something is wrong ...')

        
    


if __name__ == '__main__':
    ## User setting by GUI
    # test_setting = {'header_id': str(),
    #                 'experiment_type': str(),
    #                 'factors': list(),
    #                 'doe_type': int(),
    #                 'option': list(),}

    # test_setting['header_id']       = 'idim_test2'
    # test_setting['experiment_type'] = 'Tensile Test'
    # test_setting['factors']         = [ {'factor_name': 'infill_line_distance', 'factor_range': [1, 6]},
    #                                     {'factor_name': 'layer_height', 'factor_range': [0.1, 0.2]},
    #                                     {'factor_name': 'default_material_print_temperature', 'factor_range': [190, 220]}, ]
    # test_setting['doe_type']        = DOE_GENERALIZED_FACTORIAL # DOE_GENERALIZED_FACTORIAL=3
    # test_setting['option']          = [[1, 2, 4, 6], [0.1, 0.12, 0.2], [190, 191, 219, 220]]

    test_setting = {
                    'header_id': 'TEST',
                    'experiment_type': 'Tensile Test',
                    'factors': [ {'factor_name': 'infill_line_distance', 'factor_range': [1, 6]},
                                 {'factor_name': 'layer_height', 'factor_range': [0.1, 0.2]},
                                #  {'factor_name': 'default_material_print_temperature', 'factor_range': [190, 220]}, 
                               ],
                    'doe_type': DOE_GENERALIZED_FACTORIAL, # DOE_GENERALIZED_FACTORIAL=3
                    'option': [ [1, 2, 4, 6], 
                                [0.1, 0.12, 0.2], 
                                # [190, 191, 219, 220],
                              ],
                    }

    test = TestDesigner(test_setting)
