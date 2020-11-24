#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_SmartLAB/snu_idim_common/src"%HOME_DIR)) )

import time
from autoRun import *


if __name__ == "__main__":
    print("[DEBUG] 3DP Automation Started !!!")

    ## Automation program setting
    folder_dir = "src"
    script1 = '3DP_1_start'
    script2 = '3DP_1_stop'
    script3 = '3DP_2_start'
    script4 = '3DP_2_stop'

    time.sleep(1.0)

    ## Create an instance (initialize)
    auto3dp = idimAutomation(folder_dir)

    ## Start automation
    auto3dp.execute('{}.txt'.format(script1))
    auto3dp.execute('{}.txt'.format(script2))
    auto3dp.execute('{}.txt'.format(script3))
    auto3dp.execute('{}.txt'.format(script4))