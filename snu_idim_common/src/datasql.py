import pymysql
import pandas as pd
from PIL import Image
import base64
from io import BytesIO
import time

# ifconfig eth0 102.168.60.101


class mysql:
    def __init__(self, user=None, host=None):
        if user != None:
            self.con = pymysql.connect(user=user, host=host, port=3306, password='0000',db='SmartLab', charset='utf8')
            self.cur = self.con.cursor(pymysql.cursors.DictCursor)
            print("\n database connection success \n")
        
        else:
            print("\n database connection failed \n")


    def delete(self, db, table, specimen):
        sql1 = "USE " + db
        #example : "USE smartlab_status"
        sql2 = "DELETE TABLE " + table + " WHERE " + "subject_name=" + specimen + ";"
        #example : "DELETE TABLE Instron WHERE subject_name= specimen1;"
        
        self.cur.execute(sql1)
        self.cur.execute(sql2)

        self.con.commit()
        self.con.close()
        print("specimen deleted")

        return sql


    def sendResult(self,data_dict):
        result = data_dict
<<<<<<< Updated upstream
        subname = result['subject_name']
=======
        subject_name = result['subject_name']
>>>>>>> Stashed changes
        del(result['subject_name'])

        data_keys = result.keys()
        data_values = map(str,result.values())

        sql1 = "USE SmartLab"
        #example : "USE SmartLab"
        # print("sql1", sql1)
        self.cur.execute(sql1)
        name = "'" + subname + "'"
        sql2 = "INSERT INTO result (subject_name) VALUES (%s) ON DUPLICATE KEY UPDATE subject_name=%s;" %(name, name)
        self.cur.execute(sql2)

        sql2 = "INSERT INTO result (subject_name) VALUES (%s) ON DUPLICATE KEY UPDATE subject_name=%s;" %(subject_name,subject_name)
        self.cur.execute(sql2)


        fields = ','.join(data_keys)
        contents = "'" + "','".join(data_values) + "'"
<<<<<<< Updated upstream
        for i, field in enumerate(data_keys):
            sql3 = "UPDATE result SET %s='%s' WHERE subject_name='%s';" %(field, data_values[i], subname)
            self.cur.execute(sql3)

=======
        sql3 = "INSERT INTO result (%s) VALUES (%s) WHERE subject_name=%s" %(fields,contents,subject_name)
        # print("sql2", sql2)
        self.cur.execute(sql3)
>>>>>>> Stashed changes

        self.con.commit()
        self.con.close()
        print("\n data saved \n")



if __name__=='__main__':
    mysql = mysql()

    while True:
        pass



'''
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
'''