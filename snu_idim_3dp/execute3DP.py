#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
sys.dont_write_bytecode = True
HOME_DIR = os.getenv('HOME')
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"%s/catkin_ws/src/SNU_SmartLAB/snu_idim_common/src"%HOME_DIR)) )

import time
from autoRun import *


if __name__ == "__main__":
    folder_dir = "src"
    time.sleep(1.0)
    auto3dp = idimAutomation(folder_dir)
    auto3dp.execute('{}.txt'.format('3DP_1_start'))
    auto3dp.execute('{}.txt'.format('3DP_1_stop'))
    auto3dp.execute('{}.txt'.format('3DP_2_start'))
    auto3dp.execute('{}.txt'.format('3DP_2_stop'))