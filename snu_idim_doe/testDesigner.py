#!/usr/bin/env python
# -*- coding: utf-8 -*-

## these parameter should be imported by zeromq
TWO_LEVEL_FULL_FACTORIAL        = 1
THREE_LEVEL_FULL_FACTORIAL      = 2
GENERALIZED_FACTORIAL           = 3
TWO_LEVEL_FRACTIONAL_FACTORIAL  = 4
PLACKETT_BURMAN                 = 5
BOX_BEHNKEN                     = 6
CENTRAL_COMPOSITE               = 7

import pyDOE2 as pyDOE
import numpy as np
from DOE import DOE
import zmq
import time


class testDesigner(object):
    def __init__(self, typeOfExperiment, factors, typeOfDOE, option = False):
        self.numberOfFactors = len(factors)
        self.factors = factors
        self.typeOfDOE = typeOfDOE
        self.typeOfExperiment = typeOfExperiment

        #zeromq code
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind("tcp://*:5555")

        #PYDOE CODE
        if typeOfDOE == TWO_LEVEL_FULL_FACTORIAL:
            self.output = pyDOE.ff2n(self.numberOfFactors)
            self.output = self.output.astype(str)

        elif typeOfDOE == THREE_LEVEL_FULL_FACTORIAL:
            self.output = pyDOE.fullfact(np.ones(self.numberOfFactors, dtype = int)*3) -np.ones(self.numberOfFactors, dtype = int)

        elif typeOfDOE == GENERALIZED_FACTORIAL:
            if option is not False:
                levels = []
                for i in range(len(option)):
                    levels.append(len(option[i]))
                self.output = pyDOE.fullfact(levels)
            else:
                print('option is not defined, option should be defined as tuple')

        elif typeOfDOE == TWO_LEVEL_FRACTIONAL_FACTORIAL:
            if option is not False:
                self.output = pyDOE.fracfact(option)
                self.output = self.output.astype(str)
            else:
                print('option is not defined, option example : a b ab c ac bc abc')

        elif typeOfDOE == PLACKETT_BURMAN:
            self.output = pyDOE.pbdesign(self.numberOfFactors)

        elif typeOfDOE == BOX_BEHNKEN:
            self.output = pyDOE.bbdesign(self.numberOfFactors)

        elif typeOfDOE == CENTRAL_COMPOSITE:
            self.output = pyDOE.ccdesign(self.numberOfFactors)

        # DOE Complete with High, Low value
        self.numberOfExperiments = len(self.output)
        # print(self.output/)

        # Mapping between High, low value with actual value
        if typeOfDOE == GENERALIZED_FACTORIAL:
            for i in range(self.numberOfFactors):
                for j in range(self.numberOfExperiments):
                    self.output[j, i] = option[i][int(self.output[j, i])]

        elif typeOfDOE == TWO_LEVEL_FULL_FACTORIAL or typeOfDOE == TWO_LEVEL_FRACTIONAL_FACTORIAL:
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
                print(lower_limit, upper_limit)

                span = (upper_limit - lower_limit)/2
                offset = (upper_limit + lower_limit)/2
                self.output[:, i] = self.output[:, i] * span + offset

        self.output = np.concatenate((np.array([np.arange(self.numberOfExperiments)+1]).T, self.output), axis=1)

        # Save using Pandas Dataframe
    def saveDOE(self):
        self.save_output = DOE(self.typeOfExperiment, self.factors, self.output, self.typeOfDOE)
        return self.save_output.df

        # send the json data transformed by pandas dataframe through ZeroMQ server // UNDER DEVELOPMENT(Status check, where to send)
    def zmq_server(self):
        output_json = self.save_output.df.to_json()
        request = self.socket.recv()
        print(request.decode('utf-8'))
        time.sleep(1.0)
        try:
            # print("[DEBUG] Client requested status of devices ({})".format(request['status']))
            self.socket.send_string(output_json)
            print('[DEBUG] send complete')
        except:
            pass



if __name__ == '__main__':
# User setting by GUI
    typeOfExperiment = 'Tensile Test'
    numberOfFactors = 3
    # factors = [['THICKNESS', 0, 1], ['FEEDRATES', 0, 1], ['DENSITY', 0, 1]]
    # factors = [['AIR_GAP', 0 ,0.002], ['ROAD_WIDTH', 0.02, 0.0396], ['TEMPERATURE', 270, 280], ['COLOR', 'BLUE', 'WHITE'], ['ORIENTATION', 'TRANSVERSE', 'AXIAL']]
    # factors = [['LAYER_THICKNESS', 0.1778, 0.3556], ['ROAD_WIDTH', 0.537, 0.706], ['SPEED_DEPOSITION', 100, 200]]
    factors = [['infill_line_distance', 1, 6], ['layer_height', 0.1, 0.2], ['default_material_print_temperature', 190, 220]]
    typeOfDOE = GENERALIZED_FACTORIAL
    option = [[1,2,4,6],[0.1,0.12,0.2],[190,191,219,220]]
    # option = False

# Test Design
    test = testDesigner(typeOfExperiment, factors, typeOfDOE, option)
    test.saveDOE()
    test.zmq_server()/
