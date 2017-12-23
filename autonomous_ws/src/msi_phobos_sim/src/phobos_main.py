#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Basic framework for the autonomous operations of the MSI Phobos, the
ultra-compact rendition of the 4-wheel differential drive rover "Agastya 2.0".
Package under construction.

Copyright © 2017 by Dhruv Ilesh Shah
dhruv.shah@iitb.ac.in | dhruv.ilesh@gmail.com
The IIT-B Mars Rover Team
"""

import rospy
import std_msgs
from sensor_msgs.msg import *
import time


class gps_msg():
    # GPS Message

    def __init__(self):
        self.lat = 0.
        self.long = 0.

    def update(self, navsat_msg):
        self.lat = navsat_msg.latitude
        self.long = navsat_msg.longitude

    def read(self):
        return self.lat, self.long


def gps_callback(data):
    curr_pos.update(data)
    rospy.loginfo("GPS hit! %f, %f", data.latitude, data.longitude)
    gps_void = False


def imu_heading_callback(data):
    rospy.loginfo("IMU@UDP hit! %f", data)
    global_heading = 0.
    imu_void = False


def init_listeners():
    rospy.init_node('phobos_brain', anonymous=True)
    rospy.Subsriber("/nav_sensors/apm/gps", NavSatFix, gps_callback)
    rospy.Subsriber("/nav_sensors/imu/heading", Float64, imu_heading_callback)
    # Subscribe to joystick message! @agrim9
    drive_pub = rospy.Publisher("/drive/vel_msg", String, queue_size=10)
    drive_pub_rate = rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    curr_pos = gps_msg()
    global_heading = 0.
    gps_void = True
    imu_void = True

    usleep = lambda x: time.sleep(x / 1e6)
    while (not gps_void) and (not imu_void):
    	usleep(100)

    # Computation starts here

    init_listeners()
