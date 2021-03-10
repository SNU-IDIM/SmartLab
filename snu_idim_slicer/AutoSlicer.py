#! usr/bin/python

import sys, os, json
from threading import Thread
import zmq


class AutoSlicer():
    def __init__(self):

        ## ZMQ server setting
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind("tcp://*:6001")

        ## Test information
        self.testset_dict = dict()
        self.doe_list     = list()
        self.doe_factors  = list()

        ## Threadings ...
        thread_1 = Thread(target=self.zmqServer)
        thread_1.start()


    def zmqServer(self):
        while True:
            try:
                request = self.socket.recv()
                request_decoded = request.decode('utf-8')
                self.testset_dict = json.loads(request_decoded)
                testset_doe = self.testset_dict['doe']

                n_test = len(testset_doe['NUMBER'])

                testset_doe_keys = list(testset_doe.keys())
                testset_doe_keys.remove('NUMBER')
                self.doe_factors = testset_doe_keys
                n_factors = len(self.doe_factors)

                self.doe_list = list()
                for i in range(n_factors):
                    self.doe_list.append(list())
                    factor = self.doe_factors[i]
                    for j in range(n_test) :
                        self.doe_list[i].append(testset_doe[factor][str(j)])
                
                self.executeAutoSlicing()
                self.socket.send(b"Done")

            except:
                print("[ERROR - ZMQ] ZMQ Server Error!")


    def executeAutoSlicing(self) :

        ## Get Test Information 
        '''
        testset_dict = dict()
        testset_dict[0] = {'Number':'1' , 'infill_line_distance' : '1', 'infill_pattern' : 'lines'}
        testset_dict[1] = {'Number':'2' , 'infill_line_distance' : '6', 'infill_pattern' : 'triangles'}
        '''
        n_test_total = len(self.doe_list[0])
        n_factors    = len(self.doe_factors)

        # Change List Type Test Information to Dict Type Test Information
        testset_dict = {}
        for i in range(n_test_total):
            testset_dict[i] = {'Number':str(i)}

            for j in range(n_factors):
                testset_dict[i][self.doe_factors[j]] = self.doe_list[j][i]
        
        n_test_total = int(len(testset_dict))
        n_test_done = 0

        while n_test_done < n_test_total :

            unit_test_dict = testset_dict[n_test_done]

            ## Delete 'Number' Key to Get Factor Setting Values
            unit_test_dict.pop('Number', None) 

            n_factor_setted = 0
            n_factor_unsetted = len(unit_test_dict)

            CURA_ENGINE_DIR = os.path.join(os.getenv("HOME"), 'CuraEngine/build/CuraEngine')
            STL_FILE_DIR = "specimen_3T.stl"
            JSON_FILE_DIR = 'anet.def.json'
            TEST_HEADER_ID = self.testset_dict['header']['header_id']
            SAVE_DIR = os.path.join(os.getenv("HOME"), '.octoprint/uploads', TEST_HEADER_ID)

            try:
                os.makedirs(SAVE_DIR)
            except:
                pass

            if TEST_HEADER_ID == 'DRY_TEST': # for dry run
                print('[DEBUG] "{}_{}.gcode"'.format(TEST_HEADER_ID, n_test_done))
                f = open(os.path.join(SAVE_DIR, "{}_{}.gcode".format(TEST_HEADER_ID, n_test_done)), 'w')
                f.write('G28')
                f.close()
                n_test_done += 1
            
            else: # Automatic slicer
                command = "{} slice -v".format(CURA_ENGINE_DIR)
                option_json = ' -j "{}"'.format(JSON_FILE_DIR)
                option_save = ' -o "{}"'.format(os.path.join(SAVE_DIR, "{}_{}.gcode".format(TEST_HEADER_ID, n_test_done)))
                option_stl  = ' -l "{}"'.format(STL_FILE_DIR)

                command = command + option_json + option_save + option_stl

                while n_factor_setted < n_factor_unsetted :

                    factor_dict_keys   = list(unit_test_dict.keys())
                    factor_dict_values = list(unit_test_dict.values())

                    option_factor = ' -s "{}={}"'.format(str(factor_dict_keys[n_factor_setted]), str(factor_dict_values[n_factor_setted]))
                    command = command + option_factor

                    n_factor_setted += 1


                print("[DEBUG] Slicer command(): \n{}".format(n_test_done, command))
                os.system(command);   del command
                n_test_done += 1


if __name__ == "__main__":
    
    slicer = AutoSlicer()
