#! usr/bin/python

import sys
import os # os.system
import zmq
import json

#from Knowledge Base test info {{'Number' : '0' , 'Factor1' : '0' , 'Factor2' : '0', ... }, {'Number' : '1' , } , ...}

def zmq_client():

    context = zmq.Context()
    print("Connecting to hello world server...")
    socket = context.socket(zmq.REQ) #REQuest
    socket.connect("tcp://192.168.43.87:5555") #Change LocalHost IP 
    print('Ready')

    Request_TestInfo = dict()

    socket.send(b"Ready")
    request = socket.recv()
    Request = request.decode('utf-8')
    Request_TestInfo = json.loads(Request)

    Request_TestInfo_Keys = list(Request_TestInfo.keys())
    print(Request_TestInfo_Keys)

    Num_Test_Info= Request_TestInfo['NUMBER']
    Request_TestInfo_Keys.remove('NUMBER')
    Num_Factors = len(Request_TestInfo_Keys)
    Num_Test = len(Num_Test_Info)

    #Declare TestInfo
    TestInfo = list()
    Number = list()
    
    for i in range(Num_Factors):
        TestInfo.append(list())
        factor = Request_TestInfo_Keys[i]

        for j in range(Num_Test) :
            TestInfo[i].append(Request_TestInfo[factor][str(j)])

    return TestInfo, Request_TestInfo_Keys


def Factor2Gcode(TestInfo_List, Factor_List) :
    
    #TestInfo_2 = ['infill_sparse_thickness', 'infill_sparse_density']

    # Get Test Information 
    '''
    TestInfo = {}
    TestInfo[0] = {'Number':'1' , 'infill_line_distance' : '1', 'infill_pattern' : 'lines'}
    TestInfo[1] = {'Number':'2' , 'infill_line_distance' : '6', 'infill_pattern' : 'triangles'}
    '''
    Num_Test = len(TestInfo_List[0])
    Num_Factors = len(Factor_List)

    # Change List Type Test Information to Dict Type Test Information
    TestInfo = {}
    for i in range(Num_Test):
        TestInfo[i] = {'Number':str(i)}

        for j in range(Num_Factors):
            TestInfo[i][Factor_List[j]] = TestInfo_List[j][i]
    
    print(TestInfo)

    Test_Num = int(len(TestInfo))
    Test_Done_Num = 0

    while Test_Done_Num < Test_Num :

        TestInfo_Each = TestInfo[Test_Done_Num]

        #Delete 'Number' Key to Get Factor Setting Values
        TestInfo_Each.pop('Number', None) 

        Num_Factor_Setted = 0
        Num_Factor_Unsetted = len(TestInfo_Each)

        command = '/home/idim3d/CuraEngine/build/CuraEngine slice -v -j /home/idim3d/catkin_ws/src/SNU_SmartLAB/snu_idim_slicer/anet.def.json' 

        while Num_Factor_Setted < Num_Factor_Unsetted :

            #1. Converting Dictionary to List Datatype --> Selected Path
            ## Add "IF command" to change Children Factor when high class Factor Input##

            FactorInfo_Keys = list(TestInfo_Each.keys())
            FactorInfo_Values = list(TestInfo_Each.values())

            print(FactorInfo_Keys[Num_Factor_Setted], FactorInfo_Values[Num_Factor_Setted])

            command = command + ' -s "' + str(FactorInfo_Keys[Num_Factor_Setted]) + '=' + str(FactorInfo_Values[Num_Factor_Setted]) + '"'

            Num_Factor_Setted += 1


        Test_Done_Num += 1
        command = command + ' -o "/home/idim3d/catkin_ws/src/SNU_SmartLAB/snu_idim_3dp/gcode/test' + str(Test_Done_Num) + '.gcode"' + ' -l "/home/idim3d/catkin_ws/src/SNU_SmartLAB/snu_idim_slicer/specimen_3T.stl"' 
            
        os.system(command) #Operate
        
        print(command)

        del command

    print(TestInfo)



if __name__ == "__main__":

    TestInfo, Request_TestInfo_Keys = zmq_client()
    print(TestInfo, Request_TestInfo_Keys)
    Factor2Gcode(TestInfo, Request_TestInfo_Keys)

    print('TEST Information Successfully Changed To Gcode:)')