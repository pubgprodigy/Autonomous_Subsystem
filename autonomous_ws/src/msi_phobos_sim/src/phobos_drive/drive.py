#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from forward import *

def callback(data):
    rospy.loginfo("I heard %s", data.buttons)

    if (data.buttons[3] == 1):
        move_forward()
        rospy.loginfo("move_forward %s")

    if (data.buttons[0] == 1):
        move_back()
        rospy.loginfo("move_back %s")

    if (data.buttons[1] == 1):
        rotate_cw()
        rospy.loginfo("rotate_cw %s")

    if (data.buttons[2] == 1):
        rotate_acw()
        rospy.loginfo("rotate_acw %s")
    
    if (data.buttons[5] == 1):
        halt()
        rospy.loginfo("halt %s")

    if(data.buttons[5] == 1):
	stop()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joy_listener', anonymous=True)

    rospy.Subscriber("/joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
