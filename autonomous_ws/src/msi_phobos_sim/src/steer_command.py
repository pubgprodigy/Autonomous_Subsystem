#!/usr/bin/env python
from math import pi, cos, sin

import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline

from serial.serialutil import SerialException as SerialException

import RPi.GPIO as GPIO  # import GPIO library

GPIO.cleanup()

Motor1A = 16  # set GPIO-02 as Input 1 of the controller IC
Motor1B = 12  # set GPIO-03 as Input 2 of the controller IC
Motor2A = 26  # set GPIO-02 as Input 1 of the controller IC
Motor2B = 19  # set GPIO-03 as Input 2 of the controller IC

GPIO.setmode(GPIO.BCM)
GPIO.setup(Motor1A, GPIO.OUT)
GPIO.setup(Motor1B, GPIO.OUT)
GPIO.setup(Motor2A, GPIO.OUT)
GPIO.setup(Motor2B, GPIO.OUT)
VelPin = 18
GPIO.setup(VelPin, GPIO.OUT)
Velocity = GPIO.PWM(VelPin, 255)
Velocity.start(0)

kp1 = 2
ki1 = 2
kd1 = 2
int_windout1 = 10
sample_time = 0.1
last_time = 0.00
last_error1 = 0.00
PTerm1 = 0.00
ITerm1 = 0.00
DTerm1 = 0.00
diff_ITerm1 = 0.00
last_Ierr1 = 0.00
delta_error1 = 0.00
diff1 = 0.00
enc1Pos = 0.00
finalEnc1Val = 0.00


time_vec = []
val1_at_t = []
tval1_at_t = []
val2_at_t = []
tval2_at_t = []
plt.ion()
plt.show()


def update(headingAngle):
    global last_error1, kp1, ki1, kd1, int_windout1, sample_time, last_time, last_error1, PTerm1, ITerm1, DTerm1, diff_ITerm1, last_Ierr1, delta_error1, diff1, enc1Pos, finalEnc1Val

    current_time = time.time()
    delta_time = current_time - last_time
    #----------------------------------------------------

    if (delta_time >= 0.1):
        time_vec.append(current_time)
        enc1Pos = headingAngle
        finalEnc1Val = 30
        diff1 = finalEnc1Val - enc1Pos  # Error in 1

        delta_error1 = diff1 - last_error1
        PTerm1 = diff1  # Pterm
        ITerm1 += diff1 * delta_time


        if (ITerm1 < -int_windout1):
            ITerm1 = -int_windout1
        elif (ITerm1 > int_windout1):
            ITerm1 = int_windout1
        DTerm1 = delta_error1 / delta_time
        # Remember last time and last error for next calculation
        last_error1 = diff1
        last_Ierr1 = ITerm1

        velM1 = int((kp1 * PTerm1) + (ki1 * ITerm1) + (kd1 * DTerm1))
        rospy.loginfo(velM1);
        if enc1Pos < (finalEnc1Val - 5):
            velM1 = velM1
            val1_at_t.append(enc1Pos)
            tval1_at_t.append(headingAngle)
            GPIO.output(Motor1A, GPIO.LOW)
            GPIO.output(Motor1B, GPIO.HIGH)
            GPIO.output(Motor2A, GPIO.LOW)
            GPIO.output(Motor2B, GPIO.HIGH)
            Velocity.ChangeDutyCycle(abs(velM1))

        elif enc1Pos > (finalEnc1Val + 5):
            velM1 = velM1
            val1_at_t.append(enc1Pos)
            tval1_at_t.append(headingAngle)
            GPIO.output(Motor1A, GPIO.HIGH)
            GPIO.output(Motor1B, GPIO.LOW)
            GPIO.output(Motor2A, GPIO.HIGH)
            GPIO.output(Motor2B, GPIO.LOW)
            Velocity.ChangeDutyCycle(abs(velM1))

        else:
            GPIO.output(Motor1A, GPIO.LOW)
            GPIO.output(Motor1B, GPIO.LOW)
            GPIO.output(Motor2A, GPIO.LOW)
            GPIO.output(Motor2B, GPIO.LOW)
            Velocity.ChangeDutyCycle(abs(0))


def steer_callback(inp):
    #roboclaw1.targetAngleM1 = inp.data[6]
    rospy.loginfo(inp.data)
    headingAngle = inp.data[0] * 180 / 3.14
    update(headingAngle)


if __name__ == "__main__":

    rospy.init_node("steer_node")
    rospy.loginfo("Starting steer node")

    rospy.Subscriber("/IMU", Float32MultiArray, steer_callback)
    rospy.loginfo("I'm here")

    r_time = rospy.Rate(5)


    while(1):
         
        r_time.sleep()

