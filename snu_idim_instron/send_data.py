import sys, os
import time
import base64
from io import BytesIO

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from datasql import mysql



class bam():
    def __init__(self):
        self.subject_name = 'D30_A45_135'

        self.result = dict()
        self.result['subject_name'] = self.subject_name
        self.result['Raw_data'] = ''
        self.result['start_pic'] = ''
        self.result['finish_pic'] = ''
        self.result['Vision_data'] = ''
        self.result['plot'] = ''
        

        self.sql = mysql(user = 'IDIM-Instron', host = '192.168.60.21')
        time.sleep(2)
        self.file_name(self.subject_name)
        self.read_data()
        self.sql.sendResult(self.result)



    def file_name(self, test_name):
        last_frame = len(os.listdir('C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics')) -1
        try:
            self.raw_file = 'C:/Users/IDIM-Instron/Desktop/Smart Laboratory/' + test_name + '.is_tens_RawData/Specimen_RawData_1.csv'
            self.vision_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/' + test_name + '__vision___.xlsx'
            self.start_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics/calibrate_img0.png'
            self.finish_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/pics/calibrate_img' + str(last_frame) + '.png'
            self.plot_file = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/' + test_name + '/' + test_name + '_plot.png'

        except:
            print("file is not exist")

    def read_data(self):
        data = []
        binary_image = [0,0,0,0,0]
        with open(self.raw_file, 'rb') as r1:
            data.append(r1.read())
            r1.close()
        with open(self.vision_file, 'rb') as r2:
            data.append(r2.read())
            r2.close()
        with open(self.start_file, 'rb') as r3:
            data.append(r3.read())
            r3.close()
        with open(self.finish_file, 'rb') as r4:
            data.append(r4.read())
            r4.close()
        with open(self.plot_file, 'rb') as r5:
            data.append(r5.read())
            r4.close()

        for i in range(0,5):
            binary_image[i] = base64.b64encode(data[i])
            binary_image[i] = binary_image[i].decode('UTF-8')

        
        self.result['subject_name'] = self.subject_name
        self.result['Raw_data'] = binary_image[0]
        self.result['Vision_data'] = binary_image[1]
        self.result['start_pic'] = binary_image[2]
        self.result['finish_pic'] = binary_image[3]
        self.result['plot'] = binary_image[4]




if __name__=='__main__':
    bam = bam()




