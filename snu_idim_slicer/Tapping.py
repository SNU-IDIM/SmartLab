# Description
# Tap 9 points on the specimen by namepen
# excute with the comman '$ python3 Tapping [Filename] [Devicename]'
# For example '$ python3 Tapping /home/.octoprint/uploads/test/DRT_TEST.gcode printer1'
# x_off : the center x coordinate of the specimen with respect to namepen
# y_off : the center y coordinate of the specimen with respect to namepen
# z_off : the height right before close to the pusher
import sys

def tapping(x_off,y_off,z_off,z=8.5):
    code = 'M140 S60\nG1 E-10 F2000\nG1 X115 Y110 F1000\nG1 Z' + str(z_off) + ' F1000\nG1 Z' + str(float(z_off)+21) + ' F100\nG1 Z' + str(z_off) +  ' F1000\n'
    for i in range(-2,3,2):
        for j in range(-2,3,2):
                code = code + one_tapping(float(x_off)+i,float(y_off)+j,z)
    code = code + 'G1 X115 Y110 F1000\nG1 Z' + str(z_off) + ' F1000\nG1 Z' + str(float(z_off)+19) + ' F100\nG1 Z' + str(z_off) + 'F1000\nG1 Y220 F1000\nM17'    
    return code

def one_tapping(x, y, z):
    code = G1(x,y,z) + G1(x,y,z-3.8) + G1(x,y,z)
    return code

def G1(x,y,z):
    code = 'G1 X' + str(x) + ' Y' + str(y) + ' Z' + str(z) + '\n'
    return code

def fileOpen(location):
    f = open(location, 'a')
    return f

def fileClose(f):
    f.close()

def fileConcat(f, gCode):
    f.write(gCode)
    return f

file_name = sys.argv[1]
printer_name = sys.argv[2]

if printer_name == 'printer1':
    x_off = 108.5; y_off = 157.1; z_off = 150
elif printer_name == 'printer2':
    x_off = 108.6; y_off = 155.4; z_off = 152.6 
elif printer_name == 'printer3': 
    x_off = 110; y_off = 156; z_off = 151

# print(tapping(110.7, 157.4, 148, 8.5))
gCode = fileOpen(file_name)
gCode_tap = tapping(x_off, y_off, z_off, 8.5)
gCode = fileConcat(gCode, gCode_tap)
print(gCode_tap)
fileClose(gCode)