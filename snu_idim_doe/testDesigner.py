#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

class testDesigner(object):
    def __init__(self, factors, typeOfDOE, option = False):
        self.numberOfFactors = len(factors)
        # print(factors)

        if typeOfDOE == TWO_LEVEL_FULL_FACTORIAL:
            self.output = pyDOE.ff2n(self.numberOfFactors)
            self.output = self.output.astype(str)

        elif typeOfDOE == THREE_LEVEL_FULL_FACTORIAL:
            self.output = pyDOE.fullfact(np.ones(self.numberOfFactors, dtype = int)*3) -np.ones(self.numberOfFactors, dtype = int)

        elif typeOfDOE == GENERALIZED_FACTORIAL:
            if option is not False:
                self.output = pyDOE.fullfact(option)
                for i in range(self.numberOfFactors):
                    self.output[:, i] = (self.output[:, i] - 0.5*(option[i]-1))*2/(option[i]-1)
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

        self.numberOfExperiments = len(self.output)
        print(self.output)

        if typeOfDOE == TWO_LEVEL_FULL_FACTORIAL or typeOfDOE == TWO_LEVEL_FRACTIONAL_FACTORIAL:
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
        print(self.output)

if __name__ == '__main__':
# User setting by GUI
    typeOfExperiment = 'Tensile Test'
    numberOfFactors = 3
    # factors = [['THICKNESS', 0, 1], ['FEEDRATES', 0, 1], ['DENSITY', 0, 1]]
    # factors = [['AIR_GAP', 0 ,0.002], ['ROAD_WIDTH', 0.02, 0.0396], ['TEMPERATURE', 270, 280], ['COLOR', 'BLUE', 'WHITE'], ['ORIENTATION', 'TRANSVERSE', 'AXIAL']]
    # factors = [['LAYER_THICKNESS', 0.1778, 0.3556], ['ROAD_WIDTH', 0.537, 0.706], ['SPEED_DEPOSITION', 100, 200]]
    factors = [['RASTER_ANGLE', 0, 90], ['LAYER_THICKNESS', 0.1, 0.3], ['ROAD_WIDTH', 0.4, 0.7]]
    typeOfDOE = GENERALIZED_FACTORIAL
    option = [5,5,4]
    # option = False

# Test Design
    test = testDesigner(factors, typeOfDOE, option)
    experiment = DOE(typeOfExperiment, factors, test.output, typeOfDOE)
