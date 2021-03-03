import pymysql
import pandas as pd
from PIL import Image
import base64
from io import BytesIO
import time


class mysql():
    def __init__(self):
        
        user='IDIM-Instron'
        host='192.168.0.81'
        port = 3306
        password='0000'
        db='SNU_smartlab'
        charset='utf8'
        
        con = pymysql.connect(user=user, host=host, port=port, password=password,db=db,charset=charset)
        cur = con.cursor(pymysql.cursors.DictCursor)

# filename = open('C:/Users/IDIM-Instron/Desktop/Smart Laboratory/test9999.is_tens_RawData/Specimen_RawData_1.csv','r').read()
# filename = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/test9999/pics/calibrate_img0.png'
filename = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/test888/test888__vision___.xlsx'
with open(filename, 'rb') as f:
    # photo = f.read()
    image = f.read()

# print(image)
#-----for image/excel-----
binary_image = base64.b64encode(image)
# print(binary_image)
binary_image = binary_image.decode('UTF-8')
#-------------------

# sql = "DELETE FROM Instron;"
sql = "INSERT INTO Instron (Test_name,Result) VALUES ('Test2',"+"'" + binary_image +"'"+ ");"
# sql = str("INSERT INTO Instron Result VALUES" + photo +";")

cur.execute(sql)


sql = "SELECT Result FROM Instron;"
cur.execute(sql)
con.commit()

result = cur.fetchone()
# result = pd.DataFrame(result)
# print(result)
img = result['Result']
# print(img)
# img = img.encode('UTF-8')
img2 = base64.b64decode(img)
# print(img)
filename2 = 'C:/Users/IDIM-Instron/Desktop/SNU_SmartLAB/snu_idim_strain/result/test888/test888__vision___22.xlsx'

with open(filename2 , 'wb') as r:
    # photo = f.read()
    r.write(img2)

con.commit()
con.close()
