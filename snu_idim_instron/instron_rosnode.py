#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_device_manager")) )

from DevicePluginToROS import DevicePluginToROS
from DeviceClass_Instron import DeviceClass_Instron


if __name__ == "__main__":
    device_name = 'instron'
    rospy.init_node(device_name)
    instron_node = DevicePluginToROS(device_name=device_name, device_class=DeviceClass_Instron(device_name=device_name))