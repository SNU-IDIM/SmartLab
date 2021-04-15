import sys

def tapping(x_off,y_off,z_off,z):
    code = 'G1 E-10 F2000\nG1 X115 Y110 F1000\nG1 Z' + str(z_off) + 'F1000\nG1 Z' + str(float(z_off)+21) + 'F100\nG1 Z' + str(z_off) +  'F1000\n'
    for i in range(0,5,2):
        for j in range(0,5,2):
                code = code + one_tapping(float(x_off)+i,float(y_off)+j,z)
    code = code + 'G1 X115 Y110 F1000\nG1 Z' + str(z_off) + 'F1000\nG1 Z' + str(float(z_off)+21) + 'F100\nG1 Z' + str(z_off) + 'F1000\nG1 Y220 F1000\nM17'    
    return code

def one_tapping(x, y, z):
    code = G1(x,y,z) + G1(x,y,z-3.8) + G1(x,y,z)
    return code

def G1(x,y,z):
    code = 'G1 X' + str(x) + 'Y' + str(y) + 'Z' + str(z) + '\n'
    return code

def fileOpen(location):
    f = open(location, 'a')
    return f

def fileClose(f):
    f.close()

def fileWrite(f, gCode):
    f.write(gCode)
    return f

# print(tapping(108.7, 155.4, 148, 8.5))
gCode = fileOpen(sys.argv[1])
gCode_tap = tapping(sys.argv[2], sys.argv[3], sys.argv[4], 8.5)
gCode = fileWrite(gCode, gCode_tap)
print(gCode_tap)
fileClose(gCode)
'''
G1 E-10 F2000
G1 X115 Y110 F1000
G1 Z148 F1000
G1 Z169 F100
G1 Z148 F1000
G1 X108.7Y155.4Z8.5F2000
G1 X108.7Y155.4Z4.7
G1 X108.7Y155.4Z8.5
G1 X108.7Y157.4Z8.5F2000
G1 X108.7Y157.4Z4.7
G1 X108.7Y157.4Z8.5
G1 X108.7Y159.4Z8.5F2000
G1 X108.7Y159.4Z4.7
G1 X108.7Y159.4Z8.5
G1 X110.7Y155.4Z8.5F2000
G1 X110.7Y155.4Z4.7
G1 X110.7Y155.4Z8.5
G1 X110.7Y157.4Z8.5F2000
G1 X110.7Y157.4Z4.7
G1 X110.7Y157.4Z8.5
G1 X110.7Y159.4Z8.5F2000
G1 X110.7Y159.4Z4.7
G1 X110.7Y159.4Z8.5
G1 X112.7Y155.4Z8.5F2000
G1 X112.7Y155.4Z4.7
G1 X112.7Y155.4Z8.5
G1 X112.7Y157.4Z8.5F2000
G1 X112.7Y157.4Z4.7
G1 X112.7Y157.4Z8.5
G1 X112.7Y159.4Z8.5F2000
G1 X112.7Y159.4Z4.7
G1 X112.7Y159.4Z8.5
G1 X115 Y110 F1000
G1 Z148 F1000
G1 Z169 F100
G1 Z148 F1000
G1 Y220 F1000
M17
'''