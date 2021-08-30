import os, sys;     sys.dont_write_bytecode = True
import time
import threading
import numpy as np

from pyfirmata import Arduino, util
from pyfirmata.util import Iterator


class ArduinoInterface():
    def __init__(self, port='COM6'):

        self.board = Arduino(port)
        
        self.p_Dout = dict();   self.p_Dout['pin'] = [7, 8]
        self.p_Dout['status']  = np.zeros(len(self.p_Dout['pin']))
        self.p_Dout['command'] = np.zeros(len(self.p_Dout['pin']))

        self.p_Ain  = dict();   self.p_Ain['pin']  = [0, 1, 2, 3, 4, 5]
        self.p_Ain['status']  = np.zeros(len(self.p_Ain['pin']))

        self.p_PWM  = dict();   self.p_PWM['pin']  = [3, 5, 6, 9, 10, 11]
        self.p_PWM['status']  = np.zeros(len(self.p_PWM['pin']))
        self.p_PWM['command'] = np.zeros(len(self.p_PWM['pin']))
        self.p_PWM['cursor']  = list()


        iter = util.Iterator(self.board)
        iter.start()
        for idx, pin in enumerate(self.p_Ain['pin']):
            self.board.analog[idx].enable_reporting()

        for idx, pin in enumerate(self.p_PWM['pin']):
            self.p_PWM['cursor'].append(self.board.get_pin('d:{}:p'.format(pin)))


    def writeCommand(self, command):
        for idx, p_type in enumerate(command):
            if p_type=='p_PWM':     self.p_PWM['command'] = command['p_PWM']
            elif p_type=='p_Dout':  self.p_Dout['command'] = command['p_Dout']

    
    def readStatus(self):
        status = {'p_Ain': self.p_Ain['status'],
                  'p_Dout': self.p_Dout['status'],
                  'p_PWM': self.p_PWM['status']}
        return status


    def communicationLoop(self):
        while True:
            ## Read analog input
            for idx, pin in enumerate(self.p_Ain['pin']):
                self.p_Ain['status'][idx] = self.board.analog[pin].read()

            ## Write digital output & update status
            for idx, pin in enumerate(self.p_Dout['pin']):
                self.board.digital[pin].write(self.p_Dout['command'][idx])
                self.p_Dout['status'][idx] = self.p_Dout['command'][idx]

            ## Write PWM output & update status
            for idx, pin in enumerate(self.p_PWM['pin']):
                self.p_PWM['cursor'][idx].write(self.p_PWM['command'][idx])
                self.p_PWM['status'][idx] = self.p_PWM['command'][idx]


if __name__ == '__main__':  
    a = ArduinoInterface()
    while True:
        a.writeCommand({'p_PWM': [1, 0, 0, 0, 0, 0], 'p_Dout': [1, 0]})
        print(a.readStatus())
        time.sleep(1.)