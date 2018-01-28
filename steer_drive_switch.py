#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Bool
import math
from curtsies import Input

#import keyboard #Using module keyboard

'''
pub = rospy.Publisher('autonomous_switch', Bool,queue_size=10)
rospy.init_node('talker', anonymous=True)
while True:#making a loop
    print("Here")
    try: #used try so that if user pressed other than the given key error will not be shown
        if keyboard.is_pressed('s'):#if key 'q' is pressed 
            print('Steer Mode')
            pub.publish(True)
        elif keyboard.is_pressed('d'):
            print('Drive Mode')
            pub.publish(False)
        elif keyboard.is_pressed('q'):
            print("Quiting")
            break#finishing the loop
        else:
            pass
    except:
    #    break #if user pressed other than the given key the loop will break
        print("Exception")
'''

def main():
    pub = rospy.Publisher('autonomous_switch', Bool,queue_size=10)
    rospy.init_node('talker', anonymous=True)

    with Input(keynames='curses') as input_generator:
        for e in input_generator:
            print(str(repr(e)))
            if(str(repr(e))==str('u\'s\'')):
                print('Steer Mode')
                pub.publish(True)
            elif(str(repr(e))==str('u\'d\'')):
                print('Drive Mode')
                pub.publish(False)
            elif(str(repr(e))==str('u\'q\'')):
                print("Exiting")
                break

if __name__ == '__main__':
    main()