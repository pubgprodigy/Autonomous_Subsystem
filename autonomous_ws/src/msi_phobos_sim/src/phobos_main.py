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
from std_msgs.msg import *
from sensor_msgs.msg import *
import time
import numpy as np
import math
from msi_phobos_sim.msg import *


class gps_msg():
    # GPS Message

    def __init__(self):
        self.lat = 0.
        self.long = 0.

    def update(self, navsat_msg):
        self.lat = navsat_msg.latitude
        self.long = navsat_msg.longitude

    def update_direct(self, latit, longit):
        self.lat = latit
        self.long = longit

    def read(self):
        return self.lat, self.long

    def __add__(self, msg2):
    	[new_lat, new_long] = msg2.read()
    	[org_lat, org_long] = self.read()

    	ret = gps_msg()
    	ret.update_direct(org_lat + new_lat, org_long + new_long)
    	return ret

    def __sub__(self, msg2):
    	[new_lat, new_long] = msg2.read()
    	[org_lat, org_long] = self.read()

    	ret = gps_msg()
    	ret.update_direct(org_lat - new_lat, org_long - new_long)
    	return ret

    def __truediv__(self, const):
    	[org_lat, org_long] = self.read()
    	ret = gps_msg()
    	ret.update_direct(org_lat / const, org_long / const)
    	return ret


def gps_callback(data):
	global rolling_iter, rolling_array_size, rolling_gps, gps_void

    rolling_gps[rolling_iter].update(data)
    rolling_iter += 1
    rolling_iter %= rolling_array_size
    rospy.loginfo("GPS hit! %s, %s", data.latitude, data.longitude)

    if rolling_iter == 0:
    	gps_void = False


def imu_bearing_callback(data):
	global rover_bearing, imu_void

    rospy.loginfo("IMU@UDP hit! %s", data)
    rover_bearing = data
    imu_void = False


def drive_cc_callback(data):
	global drive_cc_status
	drive_cc_status = data
	# Pending


def get_distance(pos1, pos2):
	[lat1, long1] = pos1.read()
	[lat2, long2] = pos2.read()

	R = 6371 # Radius of the earth in km
    dLat = math.pi*(lat2 - lat1)/180
    dLon = math.pi*(long2 - long1)/180
    a = math.sin(dLat/2) * math.sin(dLat/2) +math.cos(math.pi*(lat1)/180) * math.cos(math.pi*(lat2)/180) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c # Distance in km
    
    return d


def get_bearing(pos1, pos2):
	[lat1, long1] = pos1.read()
	[lat2, long2] = pos2.read()

	dLon = (long2 - long1)
	y = math.sin(dLon) * math.cos(lat2)
	x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1)* math.cos(lat2) * math.cos(dLon)
	return math.atan2(y, x) * 180 / math.pi


def calibrate_direction(dest):
	global rover_bearing

	diff = rover_bearing - dest
	diff %= 360

	turn_dir = np.sign(diff - 180.) # +1 means acw and -1 means cw
	if diff < 180.:
		turn_steps = round(diff, -1)
	else:
		turn_steps = round(360 - diff, -1)

	vel_pub_msg = drive_control_vel()
	if not (turn_steps == 0):
		vel_pub_msg.mag = turn_steps
		if (turn_dir == 1):
			vel_pub_msg.type = "acw"
		else:
			vel_pub_msg.type = "cw"
	drive_cc_status = False
	drive_pub.publish(vel_pub_msg)
	rospy.loginfo("Turning %s degrees", turn_steps)

	# Open-loop calib done
	while not drive_cc_status:
		usleep(10 * turn_steps)

	epsilon = 1.5
	fine_diff = rover_bearing - dest
	while abs(fine_diff) > epsilon:
		# Fine tuning
		fine_diff %= 360
		fine_turn_dir = np.sign(fine_diff - 180.)
		vel_pub_msg.mag = 0. # Quantum change
		if (fine_turn_dir == 1):
			vel_pub_msg.type = "acw"
		else:
			vel_pub_msg.type = "cw"

		drive_cc_status = False
		drive_pub.publish(vel_pub_msg)
		while not drive_cc_status:
			usleep(100)

	rospy.loginfo("Calibrated to destination heading %s; Error: %s", dest, fine_diff)



def init_topics():
    rospy.init_node('phobos_brain', anonymous=True)
    rospy.Subscriber("/nav_sensors/apm/gps", NavSatFix, gps_callback)
    rospy.Subscriber("/nav_sensors/imu/bearing", Float64, imu_bearing_callback)
    rospy.Subscriber("/drive/controller_feedback", Bool, drive_cc_callback)
    # Subscribe to joystick message! @agrim9
    drive_pub = rospy.Publisher("/drive/vel_msg", drive_control_vel, queue_size=10) # fwd, bck, acw, cw, hlt
    drive_pub_rate = rospy.Rate(10)
    rospy.spin()


if __name__ == '__main__':

    init_topics()
	checkpoint_len = 0.1 # Distance (in km), of each continuous run of checkpoint
    rover_bearing = 0. # Magnetic orientation in global frame
    gps_offset = gps_msg()
    gps_void = True
    imu_void = True
    drive_cc_status = False

    rolling_array_size = 30
    rolling_gps = np.array([gps_msg() for i in range(rolling_array_size)])
    rolling_iter = 0

    dest_lat = 19.13150766
    dest_long = 72.91855635
    destination = gps_msg()
    destination.update_direct(dest_lat, dest_long)

    source_lat = 19.1311975
    source_long = 72.9186274
    source = gps_msg()
    source.update_direct(source_lat, source_long)

    usleep = lambda x: time.sleep(x / 1e6)
    while (not gps_void) and (not imu_void):
    	usleep(100)

    # Computation starts here
    curr_pos = np.sum(rolling_gps) / rolling_array_size # initial moving average
    gps_offset = start_gps - source

    dest_bearing = get_bearing(source, destination) # Ground truth measurements

    destination_dist_thresh = 10.
    run_id = 0.
    while (get_distance(source, destination) * 1000.) > destination_dist_thresh:
    	run_id += 1
    	curr_pos = np.sum(rolling_gps) / rolling_array_size # initial moving average

	    calibrate_direction(dest_bearing) # Initial calibration stage
	    vel_pub_msg = drive_control_vel()
	    vel_pub_msg.type = "fwd"
	    vel_pub_msg.mag = min(get_distance(curr_pos - gps_offset, destination), abs(checkpoint_leg)) * 1000.
	    drive_cc_status = False
	    drive_pub.publish(vel_pub_msg)
	    while not drive_cc_status:
			usleep(100)

	    rospy.loginfo("Completed run %s! Now at approx. location %s, %s", run_id, rolling_gps[rolling_iter].latitude, rolling_gps[rolling_iter].longitude)
	    
	    gps_void = False
	    rolling_iter = 0
	    while (not gps_void):
	    	usleep(100)
	    gps_void = False
	    rolling_iter = 0
	    while (not gps_void):
	    	usleep(100)
	    


