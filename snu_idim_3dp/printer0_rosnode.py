#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_device_manager")) )

from DevicePluginToROS import DevicePluginToROS
from DeviceClass_3DP import DeviceClass_3DP


if __name__ == "__main__":
    device_name = 'printer0'
    printer0_node = DevicePluginToROS(device_name=device_name, device_class=DeviceClass_3DP(device_name, ip_='localhost', port_='5001', usb_port_=0))