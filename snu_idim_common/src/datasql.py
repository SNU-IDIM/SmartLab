import pymysql
import pandas as pd
from PIL import Image
import base64
from io import BytesIO
import time

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

        user='IDIM-Instron'
        self.Device = 'Instron'        

        self.con = pymysql.connect(user=user, host='192.168.0.81', port=3306, password='0000',db='SNU_smartlab',charset='utf8')
        self.cur = con.cursor(pymysql.cursors.DictCursor)


    def file_name(self,test_name`):

        filename = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/test888/' + test_name + '__vision___.xlsx'

        return filename        


    def read_data(self, filename):
        with open(filename, 'rb') as r:
            data = r.read()

        binary_image = base64.b64encode(data)
        binary_image = binary_image.decode('UTF-8')

        return binary_image


    def save_data(self ,data1, data2):      #
        sql = "INSERT INTO" + self.Device + "(" + "Test_name" + "," + "Result" + ") VALUES (' " + data1 + "', '" + data2 +"');"
        #example : "INSERT INTO Instron (Test_name, Result) VALUES ('specimen1', '12.9mm');"

        return sql

    

    def delete_testdata(self, test_name):
        sql = "DELETE TABLE" + self.Device + "WHERE" + "Test_name=" + test_name + ";"
        #example : "DELETE TABLE Instron WHERE Test_name= specimen1;"

        return sql


    def send_data(self, test_name, result):
        file_name = self.file_name(test_name)
        bdata = self.read_data(file_name)
        save_sql = self.save_data(test_name, bdata)

        self.cur.execute(save_sql)
        
        self.con.commit()
        self.con.close()
        print("data saved")



if __name__=='__main__':
    mysql = self.mysql()

    while True:
        pass

'''
##----------------------save data ----------------------
result = cur.fetchone()
img = result['Result']
img2 = base64.b64decode(img)

filename2 = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/test888/test888__vision___22.xlsx'

with open(filename2 , 'wb') as w:
    w.write(img2)

con.commit()
con.close()
##------------------------------------------------------
'''