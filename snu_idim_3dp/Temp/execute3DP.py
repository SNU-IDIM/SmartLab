#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )

import time
from autoRun import *


if __name__ == "__main__":
    print("[DEBUG] 3DP Automation Started !!!")

    ## Automation program setting
    folder_dir = "src"
    script1 = '3DP_1_start.txt'
    script2 = '3DP_1_stop.txt'
    script3 = '3DP_2_start.txt'
    script4 = '3DP_2_stop.txt'

    time.sleep(1.0)

    ## Create an instance (initialize)
    auto3dp = idimAutomation(folder_dir)

    ## Start automation
    auto3dp.execute(script1)
    auto3dp.execute(script2)
    auto3dp.execute(script3)
    auto3dp.execute(script4)