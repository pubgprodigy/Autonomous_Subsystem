#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS Node for handling GPS data stream from the APM planner module.  
Package under construction.

Copyright © 2017 by Pranay Agarwal
The IIT-B Mars Rover Team
"""

import rospy
import math
from std_msgs.msg import String
import mavros
from sensor_msgs.msg import NavSatFix
import numpy as np


def callback(data):
    global array
    if (data.status == -1):
        rospy.logerr("error in gps data")
    else:
        gps_pub.publish(data)


def listener():
    global gps_pub

    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, callback)
    # spin() simply keeps python from exiting until this node is stopped
    gps_pub = rospy.Publisher("/nav_sensors/apm/gps", NavSatFix, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()
