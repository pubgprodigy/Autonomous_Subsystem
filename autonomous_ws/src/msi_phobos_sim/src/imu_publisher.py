#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS Node for handling UDP data stream to publish
orientation and global heading from an Android publisher.  
Package under construction.

Copyright © 2017 by Pranay Agarwal
The IIT-B Mars Rover Team
"""

import rospy,math,numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
import socket, traceback

host = ''
port = 5550

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

orientation = 1.0

def talker():
    pub = rospy.Publisher("/nav_sensors/imu/bearing", Float64, queue_size = 1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        message, address = s.recvfrom(8192)
        message = message.replace(" ","")
        orientation = np.float64(message.split(",")[-3])
        rospy.loginfo(orientation)
        pub.publish(orientation)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass