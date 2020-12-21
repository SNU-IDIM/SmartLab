#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )

from DevicePluginToROS import DevicePluginToROS
from Automate3DP import Automate3DP


if __name__ == "__main__":
    device_name = 'printer2'
    printer0_node = DevicePluginToROS(device_name=device_name, device_class=Automate3DP(device_name))