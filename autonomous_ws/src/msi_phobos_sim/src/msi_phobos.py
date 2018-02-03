#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
import time
import numpy as np
import math

#-----------------------------------------------------------------
# Class interface for GPS type messages
class gps_msg():
	# GPS Message
	#-----------------------------------------
	# Constructor
	def __init__(self):
		self.lat = 0.
		self.long = 0.

	#??????????????????????????????????????????
	#Update cords from NavSat
	def update(self, navsat_msg):
		self.lat = navsat_msg.latitude
		self.long = navsat_msg.longitude

	#-----------------------------------------
	#Update directly
	def update_direct(self, latit, longit):
		self.lat = latit
		self.long = longit

	#-----------------------------------------
	#Read cords
	def read(self):
		return self.lat, self.long

	#-----------------------------------------
	# Overloading additon
	def __add__(self, msg2):
		[new_lat, new_long] = msg2.read()
		[org_lat, org_long] = self.read()

		ret = gps_msg()
		ret.update_direct(org_lat + new_lat, org_long + new_long)
		return ret

	# Overloading subtraction
	def __sub__(self, msg2):
		[new_lat, new_long] = msg2.read()
		[org_lat, org_long] = self.read()

		ret = gps_msg()
		ret.update_direct(org_lat - new_lat, org_long - new_long)
		return ret

	# Overloading scalar division
	def __truediv__(self, const):
		[org_lat, org_long] = self.read()
		ret = gps_msg()
		ret.update_direct(org_lat / const, org_long / const)
		return ret

#-----------------------------------------------------------------
# Callback function for LatLon publisher
def gps_callback(inp):
	#-------------------------------------------------
	# Declare what variables should be in global scope
	global rolling_iter, rolling_gps, gps_void
	#-------------------------------------------------
	# Get data from the publisher
	data = gps_msg()
	data.latitude = inp.data[0]
	data.longitude = inp.data[1]
	#rospy.loginfo("GPS hit! %s, %s", data.latitude, data.longitude)
	#-------------------------------------------------
	# Start filling the rolling array
	rolling_gps[rolling_iter].update(data)
	rolling_iter += 1
	rolling_iter %= rolling_array_size
	# Indicate absence of GPS readings if rolling array is void
	if rolling_iter == 0:
		gps_void = False
	#-------------------------------------------------

#-----------------------------------------------------------------
# Callback function for IMU publisher
def imu_bearing_callback(inp):
	#-------------------------------------------------
	# Declare what variables should be in global scope
	global rover_bearing, imu_void
	#rospy.loginfo("IMU hit! %s", inp.data[0] * 180.0/np.pi)
	# Update Rover bearing
	rover_bearing = inp.data[0] * 180.0/np.pi
	# Indicate that IMU sensor is not void
	imu_void = False
#-----------------------------------------------------------------

#-----------------------------------------------------------------
# Callback function for mobility_feedback
def mobility_callback(inp):
	global mobility_status
	# Mobility status of -1 indicated stopping of steer (Can be made bool as well)
	mobility_status =  inp.data
	print("Rxed:"+str(mobility_status))
#-----------------------------------------------------------------

#-----------------------------------------------------------------
# Function to get distance between 2 gps_cords (From SO)
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

#-----------------------------------------------------------------
# Function to get heading between 2 gps_cords (From SO)
def get_bearing(pos1, pos2):
	[lat1, long1] = pos1.read()
	[lat2, long2] = pos2.read()

	dLon = (long2 - long1)
	y = math.sin(dLon) * math.cos(lat2)
	x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1)* math.cos(lat2) * math.cos(dLon)
	return math.atan2(y, x) * 180 / math.pi
#-----------------------------------------------------------------

#-----------------------------------------------------------------
# Function to callibrate the heading of the rover wrt dest
def calibrate_direction(dest):
	#-------------------------------------------------
	# Declare what variables should be in global scope
	global rover_bearing,ang_pub,mobility_status
	#-------------------------------------------------
	# Get the angle by which rover needs to turn
	diff = dest
	#-------------------------------------------------
	# Set mobility status to 0 before proceeding
	mobility_status = 0
	#-------------------------------------------------
	# Command rover to turn by diff degrees
	print("Commanded Rover to turn by "+str(diff))
	ang_pub.publish(diff)
	#-------------------------------------------------
	# Wait till steer has ended
	while True:
		if(mobility_status==-1):
			print("End steer Rxed!!")
			break
	# Safety guard band of 100ms
	usleep(100000)
	#-------------------------------------------------
	# Print the diff error
	final_diff = rover_bearing - dest	
	rospy.loginfo("Calibrated to destination heading %s; Error: %s", dest, final_diff)
	#-------------------------------------------------

if __name__ == '__main__':

	#-----------------------------------------------------------------
	# Create Subscribers
	rospy.Subscriber("LatLon", Float64MultiArray, gps_callback)
	rospy.Subscriber("IMU", Float32MultiArray, imu_bearing_callback)
	rospy.Subscriber("mobility_feedback", Int32, mobility_callback)
	#-----------------------------------------------------------------
	# Create Publishers
	ang_pub = rospy.Publisher("/ang_msg", Float64, queue_size=10) 
	drive_pub = rospy.Publisher("/drive_msg", Float64, queue_size=10)
	#-----------------------------------------------------------------
	# Create ROS Node
	rospy.init_node('phobos_brain', anonymous=True)
	#-----------------------------------------------------------------
	# Declare Global variables with their default values
	#---------------------
	# Pub-Sub vars
	checkpoint_len = 0.05 # Distance (in km), of each continuous run of checkpoint
	rover_bearing = 0. # Magnetic orientation in global frame
	gps_offset = gps_msg()
	gps_void = True
	imu_void = True
	mobility_status = 0
	rolling_array_size = 5
	rolling_gps = np.array([gps_msg() for i in range(rolling_array_size)])
	rolling_iter = 0
	#---------------------
	# Set up destination
	dest_lat = 19.13141702
	dest_long = 72.91837903
	destination = gps_msg()
	destination.update_direct(dest_lat, dest_long)
	#---------------------
	# Set up source
	source_lat = 19.13109811
	source_long = 72.9182855
	source = gps_msg()
	source.update_direct(source_lat, source_long)
	#---------------------
	# Declare global micro seconds sleep lamda
	usleep = lambda x: time.sleep(x / 1e6)
	#-----------------------------------------------------------------
	# Wait for GPS, IMU publishers to publish
	while (not gps_void) and (not imu_void):
		usleep(100)
	#-----------------------------------------------------------------
	# Get the current position
	curr_pos = gps_msg()
	curr_pos = np.sum(rolling_gps) / rolling_array_size #moving average
	#---------------------
	# Get the initial offset
	gps_offset = curr_pos - source
	#---------------------
	# Get initial heading angle
	dest_bearing = get_bearing(source, destination) # Ground truth measurements
	#---------------------
	# Declare threshold
	destination_dist_thresh = 5.
	#---------------------
	# Declare loop control variables
	run_id = 0.
	start=1
	curr_pos = np.sum(rolling_gps) / rolling_array_size

	#-----------------------------------------------------------------
	# Start the process
	while True:
		#------------------------------------------------------------
		# Increment run_id counter
		run_id += 1
		#------------------------------------------------------------
		#Callibrate in start or when required
		if(start==1):
			calibrate_direction(dest_bearing) # Calibrate if required stage
			if(start==1):
				start=0
		#------------------------------------------------------------
		#Get the current distance magnitude left to cover
		distance_mag = get_distance(curr_pos - gps_offset, destination)*1000
		#------------------------------------------------------------
		#Stop if within threshold
		if(distance_mag < destination_dist_thresh):
			print("Target Reached, start camera node")
			break
		#------------------------------------------------------------
		#Inform steer drive code about how much yet to go
		drive_pub.publish(distance_mag)
		rospy.loginfo("Commanded to move distance = %s", distance_mag)
		#------------------------------------------------------------
		# Wait for new set of GPS before proceeding
		gps_void = True
		rolling_iter = 0
		while (not gps_void):
			usleep(100)
		#------------------------------------------------------------
		# Update the current Pos
		curr_pos = np.sum(rolling_gps) / rolling_array_size # moving average
	#-----------------------------------------------------------------
	# END of CODE :D
	rospy.loginfo("Completed run %s! Now at approx. location %s, %s", run_id, rolling_gps[rolling_iter].latitude, rolling_gps[rolling_iter].longitude)
		


