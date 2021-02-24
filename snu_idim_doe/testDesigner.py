#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import zmq
from DOE import DOE
import numpy as np
import pyDOE2 as pyDOE
import json


TWO_LEVEL_FULL_FACTORIAL = 1
THREE_LEVEL_FULL_FACTORIAL = 2
GENERALIZED_FACTORIAL = 3
TWO_LEVEL_FRACTIONAL_FACTORIAL = 4
PLACKETT_BURMAN = 5
BOX_BEHNKEN = 6
CENTRAL_COMPOSITE = 7


class testDesigner(object):

    def __init__(self, test_setting, ip='192.168.0.81'):
        ## Subject Info. definition
        self.header_ = test_setting
        self.info_ = {'experiment_sets': dict(),
                      'experiment_status': list(),}
                      
        self.typeOfExperiment = self.header_['experiment_type']
        self.factors = self.header_['factors']
        self.numberOfFactors = len(self.header_['factors'])
        self.typeOfDOE = self.header_['doe_type']

        # ## ZMQ setting (This node is a server side)
        # self.context = zmq.Context()
        # self.socket = self.context.socket(zmq.REP)
        # self.socket.bind("tcp://*:5555")

        ## ZMQ setting (This node is a client side)
        try:
            self.context = zmq.Context()
            print("[DEBUG] ZMQ - Connecting...")
            self.socket = self.context.socket(zmq.REQ) # REQuest
            self.socket.connect("tcp://{}:5555".format(ip)) # Change LocalHost IP 
            print("[DEBUG] ZMQ - Client initialized!")
        except:
            print("[ERROR] ZMQ error")

        self.makeDOE()
        self.saveDOE()
        self.zmq_server()


    def makeDOE(self):
        if self.typeOfDOE == TWO_LEVEL_FULL_FACTORIAL:
            self.output = pyDOE.ff2n(self.numberOfFactors)
            self.output = self.output.astype(str)

        elif self.typeOfDOE == THREE_LEVEL_FULL_FACTORIAL:
            self.output = pyDOE.fullfact(np.ones(
                self.numberOfFactors, dtype=int)*3) - np.ones(self.numberOfFactors, dtype=int)

        elif self.typeOfDOE == GENERALIZED_FACTORIAL:
            if self.header_['option'] is not False:
                levels = []
                for i in range(len(self.header_['option'])):
                    levels.append(len(self.header_['option'][i]))
                self.output = pyDOE.fullfact(levels)
                # print(self.output)
            else:
                print('option is not defined, option should be defined as tuple')

        elif self.typeOfDOE == TWO_LEVEL_FRACTIONAL_FACTORIAL:
            if self.header_['option'] is not False:
                self.output = pyDOE.fracfact(option)
                self.output = self.output.astype(str)
            else:
                print('option is not defined, option example : a b ab c ac bc abc')

        elif self.typeOfDOE == PLACKETT_BURMAN:
            self.output = pyDOE.pbdesign(self.numberOfFactors)

        elif self.typeOfDOE == BOX_BEHNKEN:
            self.output = pyDOE.bbdesign(self.numberOfFactors)

        elif self.typeOfDOE == CENTRAL_COMPOSITE:
            self.output = pyDOE.ccdesign(self.numberOfFactors)

        ## DOE Complete with High, Low value
        self.numberOfExperiments = len(self.output)
        # print(self.output)

        ## Mapping between High, low value with actual value
        if self.typeOfDOE == GENERALIZED_FACTORIAL:
            for i in range(self.numberOfFactors):
                for j in range(self.numberOfExperiments):
                    self.output[j, i] = self.header_['option'][i][int(self.output[j, i])]

        elif self.typeOfDOE == TWO_LEVEL_FULL_FACTORIAL or self.typeOfDOE == TWO_LEVEL_FRACTIONAL_FACTORIAL:
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


    ## Save using Pandas Dataframe
    def saveDOE(self):
        test_sets = DOE(self.typeOfExperiment, self.factors, self.output, self.typeOfDOE)
        self.test_sets = test_sets.df
        # print(len(test_sets.df.T.to_dict()), type(test_sets.df.T.to_dict()), test_sets.df.T.to_dict())
        for id_number in test_sets.df.T.to_dict().keys():
            print(self.header_['header_id'] + '_' + str(id_number), test_sets.df.T.to_dict()[id_number])
        return self.test_sets


    ## send the json data transformed by pandas dataframe through ZeroMQ server // UNDER DEVELOPMENT(Status check, where to send)
    def zmq_server(self):
        # request = self.socket.recv()
        # time.sleep(1.0)
        # try:
        #     testset_dict = dict()
        #     testset_dict['header'] = self.header_
        #     testset_dict['doe'] = self.test_sets.to_dict()
        #     print(testset_dict)
        #     self.socket.send_string(json.dumps(testset_dict))
        #     print("[DEBUG - zmq_server] 'Test set' was sent to the client successfully !")
        # except:
        #     print("[ERROR - zmq_server]")
        #     pass
        try:
            testset_dict = dict()
            testset_dict['header'] = self.header_
            testset_dict['doe'] = self.test_sets.to_dict()
            print(testset_dict)
            self.socket.send_string(json.dumps(testset_dict))
            response = self.socket.recv()
            print(response)
        except:
            print('[ERROR - ZMQ Client]')


# def returnFactorInfo(factor_name='', factor_range=[]):
#     factor_info = {'factor_name': factor_name,
#                    'factor_range': factor_range}
#     return factor_info


if __name__ == '__main__':
    ## User setting by GUI
    test_setting = {'header_id': str(),
                    'experiment_type': str(),
                    'factors': list(),
                    'doe_type': int(),
                    'option': list(),}

    test_setting['header_id']       = 'idim_test2'
    test_setting['experiment_type'] = 'Tensile Test'
    test_setting['factors']         = [ {'factor_name': 'infill_line_distance', 'factor_range': [1, 6]},
                                        {'factor_name': 'layer_height', 'factor_range': [0.1, 0.2]},
                                        {'factor_name': 'default_material_print_temperature', 'factor_range': [190, 220]}, ]
    test_setting['doe_type']        = 3
    test_setting['option']          = [[1, 2, 4, 6], [0.1, 0.12, 0.2], [190, 191, 219, 220]]


    # test_setting['factors'] = [returnFactorInfo(factor_name='infill_line_distance', factor_range=[1, 6]),
    #                            returnFactorInfo(factor_name='layer_height', factor_range=[0.1, 0.2]),
    #                            returnFactorInfo(factor_name='default_material_print_temperature', factor_range=[190, 220]),]
        

    ## Test Design
    test = testDesigner(test_setting)
