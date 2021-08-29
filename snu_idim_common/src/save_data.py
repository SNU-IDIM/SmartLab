import pymysql
import pandas as pd
from PIL import Image
import base64
from io import BytesIO
import time
import json
import os, sys;     sys.dont_write_bytecode = True

##-------------------------------------------------------------------------------------------
##-----------------------------------need to change------------------------------------------
##-------------------------------------------------------------------------------------------
##-----------1. __init__ : user, Device------------------------------------------------------
##-----------2. file_name : file directory --------------------------------------------------
##-----------3. save_data :  number of input data--------------------------------------------
##-----------4. delet_data : name of table line title----------------------------------------
##-------------------------------------------------------------------------------------------
##-------------------------------------------------------------------------------------------
##-------------------------------------------------------------------------------------------

class mysql():
    def __init__(self):

        user='root'
        self.Device = 'idim3D'        

        self.con = pymysql.connect(user=user, host='localhost', port=3306, password='0000',db='SmartLab',charset='utf8',cursorclass = pymysql.cursors.DictCursor)
        # self.cur = self.con.cursor(pymysql.cursors.DictCursor)
        self.cur = self.con.cursor()
        self.save_data('calb')
        print("f")




    def read_data(self, filename):
        with open(filename, 'rb') as r:
            data = r.read()

        binary_image = base64.b64encode(data)
        binary_image = binary_image.decode('UTF-8')

        return binary_image


    

    def delete_testdata(self,Device, test_name):
        sql = "DELETE TABLE" + self.Device + "WHERE" + "Test_name='" + test_name + "';"
        #example : "DELETE TABLE Instron WHERE Test_name='specimen1';"

        return sql


    def save_data(self, test_name):
        sql1 = "SELECT * FROM result WHERE subject_name='" + test_name + "';"
        #example : "SELECT * FROM Instron WHERE Test_naame='specimen1';"
        self.cur.execute(sql1)
        result = self.cur.fetchall()
        # result = dict(result)
        # print(result['Test_name'])
        # print(type(result))
        result = result[0]
        # print(type(result))
        # df = pd.DataFrame(result)

        for k in result.keys():
            # print(k,result[k])
            newpath = './result/Instron/'+ test_name
            
            if not os.path.exists(newpath):
                os.makedirs(newpath)
                print("made_directory=",newpath)
            print(k)
            
            if k == 'subject_name' or k == 'Thickness' or k == 'Length' or k == 'Width':
                continue
            elif k == 'Raw_data':
                filename = newpath + '/' + k +'.csv'

            elif k == 'Vision_data':
                filename = newpath + '/' + k +'.xlsx'

            elif k == 'start_pic':
                filename = newpath + '/' + k +'.png'

            elif k == 'finish_pic':
                filename = newpath + '/' + k +'.png'
            
            elif k == 'plot':
                filename = newpath + '/' + k +'.png'
            
                
            # print('filename',filename)
            
            with open(filename, 'wb') as w:
                recieved = base64.b64decode(result[k])
                w.write(recieved)


        self.con.commit()
        self.con.close


if __name__=='__main__':
    mysql = mysql()

    while True:
        pass
