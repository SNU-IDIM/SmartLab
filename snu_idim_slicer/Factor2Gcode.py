#! usr/bin/python

import sys
import os # os.system
import zmq
import json

class AutoSlicer():
    def __init__(self, ip='192.168.60.25'):
        try:
            self.context = zmq.Context()
            print("[DEBUG] ZMQ - Connecting...")
            self.socket = self.context.socket(zmq.REQ) # REQuest
            self.socket.connect("tcp://{}:5555".format(ip)) # Change LocalHost IP 
            print("[DEBUG] ZMQ - Connected to server!")
        except:
            print("[ERROR] ZMQ error")

        self.testset_list    = list()
        self.testset_factors = list()

    
    def sendRequest(self):
        testset_dict = dict()

        self.socket.send(b"Ready")
        request = self.socket.recv()

        request_decoded = request.decode('utf-8')
        testset_dict = json.loads(request_decoded)
        

        n_test = len(testset_dict['NUMBER'])

        testset_dict_keys = list(testset_dict.keys())
        testset_dict_keys.remove('NUMBER')
        self.testset_factors = testset_dict_keys
        print(self.testset_factors)

        n_factors = len(self.testset_factors)

        self.testset_list = list()
        for i in range(n_factors):
            self.testset_list.append(list())
            factor = self.testset_factors[i]

            for j in range(n_test) :
                self.testset_list[i].append(testset_dict[factor][str(j)])




    def execute(self) :

        ## Get Test Information 
        '''
        testset_dict = {}
        testset_dict[0] = {'Number':'1' , 'infill_line_distance' : '1', 'infill_pattern' : 'lines'}
        testset_dict[1] = {'Number':'2' , 'infill_line_distance' : '6', 'infill_pattern' : 'triangles'}
        '''
        n_test    = len(self.testset_list[0])
        n_factors = len(self.testset_factors)

        # Change List Type Test Information to Dict Type Test Information
        testset_dict = {}
        for i in range(n_test):
            testset_dict[i] = {'Number':str(i)}

            for j in range(n_factors):
                testset_dict[i][self.testset_factors[j]] = self.testset_list[j][i]
        
        print(testset_dict)

        n_test = int(len(testset_dict))
        n_test_done = 0

        while n_test_done < n_test :

            unit_test_dict = testset_dict[n_test_done]

            ## Delete 'Number' Key to Get Factor Setting Values
            unit_test_dict.pop('Number', None) 

            n_factor_setted = 0
            n_factor_unsetted = len(unit_test_dict)

            HOME_DIR = os.getenv("HOME")
            CURA_ENGINE_DIR = os.path.join(HOME_DIR, 'CuraEngine/build/CuraEngine')
            STL_FILE_DIR = "specimen_3T.stl"
            JSON_FILE_DIR = 'anet.def.json'
            TEST_HEADER_ID = 'demo'
            SAVE_DIR = os.path.join(HOME_DIR, '.octoprint/uploads', TEST_HEADER_ID)

            try:
                os.makedirs(SAVE_DIR)
            except:
                pass

            ## Slicer command
            command = "{} slice -v".format(CURA_ENGINE_DIR)
            option_json = ' -j "{}"'.format(JSON_FILE_DIR)
            option_save = ' -o "{}"'.format(os.path.join(SAVE_DIR, "{}_{}.gcode".format(TEST_HEADER_ID, n_test_done)))
            option_stl  = ' -l "{}"'.format(STL_FILE_DIR)

            command = command + option_json

            while n_factor_setted < n_factor_unsetted :

                #1. Converting Dictionary to List Datatype --> Selected Path
                ## Add "IF command" to change Children Factor when high class Factor Input##

                factor_dict_keys   = list(unit_test_dict.keys())
                factor_dict_values = list(unit_test_dict.values())

                option_factor = ' -s "{}={}"'.format(str(factor_dict_keys[n_factor_setted]), str(factor_dict_values[n_factor_setted]))
                command = command + option_factor

                n_factor_setted += 1

            n_test_done += 1
            
            command = command + option_save + option_stl
            print("[DEBUG] Slicer command: \n{}".format(command))

            os.system(command) #Operate
            

            del command

        print(testset_dict)



if __name__ == "__main__":
    
    slicer = AutoSlicer(ip='192.168.0.40')
    slicer.sendRequest()
    slicer.execute()

    # TestInfo, Request_TestInfo_Keys = zmq_client()
    # print(TestInfo, Request_TestInfo_Keys)
    # Factor2Gcode(TestInfo, Request_TestInfo_Keys)

    # print('TEST Information Successfully Changed To Gcode:)')