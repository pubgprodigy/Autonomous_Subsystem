#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from forward import *
import RPi.GPIO as GPIO # import GPIO library
#from time import sleep

global orientation
global target_orientation
target_orientation = 0
global epsilon
epsilon = 5

# def joy_callback(data):
#     rospy.loginfo("I heard %s", data.buttons)

#     if (data.buttons[3] == 1):
#         move_forward()
#         rospy.loginfo("move_forward %s")

#     if (data.buttons[0] == 1):
#         move_back()
#         rospy.loginfo("move_back %s")

#     if (data.buttons[1] == 1):
#         rotate_cw()
#         rospy.loginfo("rotate_cw %s")

#     if (data.buttons[2] == 1):
#         rotate_acw()
#         rospy.loginfo("rotate_acw %s")
    
#     if (data.buttons[5] == 1):
#         halt()
#         rospy.loginfo("halt %s")


def drive_callback(data):
    if(data.data < 345 and data.data > 180):
#        rospy.loginfo("turning acw", data)
	print("turning acw")
	rotate_acw()

    if(data.data < 180 and data.data > 15):
#        rospy.loginfo("turning cw", data)
	print("turning cw")
	rotate_cw()

    if(data.data < 15 or data.data > 345):
#       	rospy.loginfo("halt", data)
	print("halt")
	halt()

    print(data.data)

if __name__ == '__main__':
    print "Start"
#    listener()
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joy_listener', anonymous=True)

    # rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.Subscriber("/nav_sensors/imu/bearing", Float64, drive_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

