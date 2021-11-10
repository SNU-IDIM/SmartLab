import pyfirmata as pf

import time


ard = pf.Arduino('COM6')

ard.get_pin('d:13:o') #디지털핀 13번을 출력으로 설정

while True:

    ard.digital[7].write(1)

    time.sleep(0.5)

    ard.digital[7].write(0)

    time.sleep(0.5)


# import pyfirmata as pf

# import time


# ard = pf.Arduino('COM6')

# print('connected.')


# # 아날로그핀을 사용하려면 반드시 아래와 같은 두 줄이 필요함.

# pf.util.Iterator(ard).start()

# ard.analog[0].enable_reporting()


# while True:

# 	a0 = ard.analog[0].read() # [0,1]범위의 실수 반환

# 	print(a0)