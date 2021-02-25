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
        self.cur = self.con.cursor(pymysql.cursors.DictCursor)
        # self.send_data('test888')



    def file_name(self,last_frame,test_name):
        try:
            self.raw_file = 'C:/Users/IDIM-Instron/Desktop/Smart Laboratory/' + test_name + '.is_tens_RawData/Specimen_RawData_1.csv'
            self.vision_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/' + test_name + '__vision___.xlsx'
            self.start_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics/calibrate_img0.png'
            self.finish_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics/calibrate_img' + str(last_frame) + '.png'

        except:
            print("file is not exist")


    def read_data(self, filename):
        with open(filename, 'rb') as r:
            data = r.read()

        binary_image = base64.b64encode(data)
        binary_image = binary_image.decode('UTF-8')

        return binary_image


    def save_data(self ,test_name ,data1, data2, data3, data4):      #
        sql = "INSERT INTO " + self.Device +  " VALUES ('" + test_name + "','" + data1 + "','" + data2 + "','" + data3 + "','" + data4 + "');"
        #example : "INSERT INTO Instron VALUES ('specimen1','data1','data2','data3','data4');"

        return sql

    

    def delete_testdata(self, test_name):
        sql = "DELETE TABLE" + self.Device + "WHERE" + "Test_name=" + test_name + ";"
        #example : "DELETE TABLE Instron WHERE Test_name= specimen1;"

        return sql


    def send_data(self,last_frame, test_name):
        self.file_name(last_frame, test_name)
        bdata1 = self.read_data(self.raw_file)
        bdata2 = self.read_data(self.vision_file)
        bdata3 = self.read_data(self.start_file)
        bdata4 = self.read_data(self.finish_file)
        
        save_sql = self.save_data(test_name, bdata1, bdata2, bdata3, bdata4)


        self.cur.execute(save_sql)
        
        self.con.commit()
        self.con.close()
        print("data saved")



if __name__=='__main__':
    mysql = mysql()

    while True:
        pass
