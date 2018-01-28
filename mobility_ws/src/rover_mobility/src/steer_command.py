#!/usr/bin/env python
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
from roboclaw import RoboClaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,Float32MultiArray,Bool
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline

from serial.serialutil import SerialException as SerialException

fin_ang=50
dir1=1
dir2=1
speed=0
mode="drive"
class SteerClaw:

    def __init__(self, address, dev_name, baud_rate, name, kp1 = 0.7, kp2=0.7,ki1=0.2,ki2=0.2,kd1=30,kd2=30,int_windout1=20,int_windout2=20, qpps1 = 1, qpps2 = 1, deadzone1 = 3, deadzone2 = 3, kon1 = 0, kon2 = 0, sample_time=0.01, last_time=0.00, curren_time=0.00):
        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
        0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
        0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
        0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
        0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
        0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
        0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
        0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
        0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
        0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
        0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
        0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
        0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
        0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
        0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
        0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
        0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        self.claw = RoboClaw(address, dev_name, baud_rate)
        self.name = name
        self.claw.ResetEncoders()
        self.targetAngleM1 = 0
        self.targetAngleM2 = 0
        self.kp1 = kp1
        self.kp2 = kp2
        self.ki1 = ki1
        self.ki2 = ki2
        self.kd1 = kd1
        self.kd2 = kd2
        self.qpps1 = qpps1
        self.qpps2 = qpps2
        self.deadzone1 = deadzone1
        self.deadzone2 = deadzone2
        self.int_windout1=int_windout1
        self.int_windout2=int_windout2
        self.kon1 = kon1
        self.kon2 = kon2
        self.sample_time = sample_time
        self.last_time = 0.00
        self.last_error1 = 0.00
        self.last_error2 = 0.00
        self.PTerm1=0.00
        self.ITerm1=0.00
        self.DTerm1=0.00
        self.PTerm2=0.00
        self.ITerm2=0.00
        self.DTerm2=0.00
        self.diff_ITerm1=0.00
        self.diff_ITerm2=0.00
        self.last_Ierr1=0.00
        self.last_Ierr2=0.00
        self.delta_error1=0.00
        self.delta_error2=0.00
        self.diff1=0.00
        self.diff2=0.00
        self.enc1Pos=0.00
        self.enc2Pos=0.00
        self.finalEnc1Val=0.00
        self.finalEnc2Val=0.00
        self.initialAngleM1=0
        self.initialAngleM2=0

    def update(self):
        global fin_ang
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        #----------------------------------------------------
        #Roboclaw1
        if (delta_time >= self.sample_time):
            self.enc1Pos = self.initialAngleM1
            self.finalEnc1Val = fin_ang
                        #rospy.loginfo(self.finalEnc1Val-self.enc1Pos)
            self.diff1 = self.finalEnc1Val - self.enc1Pos  #Error in 1
            
            self.delta_error1 = self.diff1 - self.last_error1
            self.PTerm1 = self.diff1 #Pterm
            self.ITerm1+=self.diff1*delta_time
            
            if(self.last_Ierr1!=0):
                self.diff_ITerm1=self.ITerm1-self.last_Ierr1
            
            if (self.ITerm1 < -self.int_windout1):
                self.ITerm1 = -self.int_windout1
            elif (self.ITerm1 > self.int_windout1):
                self.ITerm1 = self.int_windout1
            self.DTerm1 = self.delta_error1 / delta_time
            # Remember last time and last error for next calculation
            self.last_error1 = self.diff1
            self.last_Ierr1=self.ITerm1

            velM1 = int((self.kp1*self.PTerm1) + (self.ki1 * self.ITerm1) + (self.kd1 * self.DTerm1))

            if self.enc1Pos < (self.finalEnc1Val - self.deadzone1):
                velM1 = velM1 + self.kon1
                if(self.name=="forward"):
                    self.claw.BackwardM1(min(255, velM1))
                else:
                    self.claw.ForwardM1(min(255, velM1))

            elif self.enc1Pos > (self.finalEnc1Val + self.deadzone1):
                velM1 = velM1 - self.kon1
                if(self.name=="forward"):
                    self.claw.ForwardM1(min(255, -velM1))
                else:
                    self.claw.BackwardM1(min(255, -velM1))
            else:
                self.claw.ForwardM1(0)



            self.enc2Pos = self.initialAngleM2
            self.finalEnc2Val = fin_ang
            self.diff2 = self.finalEnc2Val - self.enc2Pos  #Error in 1
            self.delta_error2 = self.diff2 - self.last_error2
            self.PTerm2 = self.diff2 #Pterm
            self.ITerm2+=self.diff2*delta_time
           
            if(self.last_Ierr2!=0):
                self.diff_ITerm2=self.ITerm2-self.last_Ierr2

            if (self.ITerm2 < -self.int_windout2):
                self.ITerm2 = -self.int_windout2
            elif (self.ITerm2 > self.int_windout2):
                self.ITerm2 = self.int_windout2

            self.DTerm2 = 0.0
            if delta_time > 0:
                self.DTerm2 = self.delta_error2 / delta_time

            # Remember last time and last error for next calculation
            self.last_error2 = self.diff2
            self.last_Ierr2=self.ITerm2
            velM2 = int((self.kp2*self.PTerm2) + (self.ki2 * self.ITerm2) + (self.kd2 * self.DTerm2))


            if self.enc2Pos < (self.finalEnc2Val - self.deadzone2):
                velM2 = velM2 + self.kon2
                if(self.name=="forward"):
                    self.claw.BackwardM2(min(255, velM2))
                else:
                    self.claw.BackwardM2(min(255, velM2))

            elif self.enc2Pos > (self.finalEnc2Val + self.deadzone2):
                velM2 = velM2 - self.kon2
                if(self.name=="forward"):
                    self.claw.ForwardM2(min(255, -velM2))
                else:
                    self.claw.ForwardM2(min(255, -velM2))

            else:
                self.claw.ForwardM2(0)
            print("Final Stopping Angle: "+str(fin_ang)+" Enc1Pos:"+str(self.enc1Pos)+" Enc2Pos:"+str(self.enc2Pos))
            
            
    
def moveFwd():
    roboclaw1.claw.BackwardM1(min(255,int(speed)))
    roboclaw1.claw.BackwardM2(min(255,int(speed)))    
    roboclaw2.claw.ForwardM1(min(255,int(speed)))
    roboclaw2.claw.BackwardM2(min(255,int(speed)))    

def moveBkwd():
    roboclaw1.claw.ForwardM1(min(255,int(speed)))
    roboclaw1.claw.ForwardM2(min(255,int(speed)))    
    roboclaw2.claw.BackwardM1(min(255,int(speed)))
    roboclaw2.claw.ForwardM2(min(255,int(speed)))    
    
def moveL():
    roboclaw1.claw.BackwardM1(min(255,int(speed)))
    roboclaw1.claw.ForwardM2(min(255,int(speed)))
    roboclaw2.claw.ForwardM1(min(255,int(speed)))
    roboclaw2.claw.ForwardM2(min(255,int(speed)))

def moveR():
    roboclaw1.claw.ForwardM1(min(255,int(speed)))
    roboclaw1.claw.BackwardM2(min(255,int(speed)))    
    roboclaw2.claw.BackwardM1(min(255,int(speed)))
    roboclaw2.claw.BackwardM2(min(255,int(speed)))

def rest():
    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)    
    roboclaw2.claw.ForwardM1(0)
    roboclaw2.claw.ForwardM2(0)   
 
def update_steer():
    #global dir1,dir2
    if(dir1==0 or dir2==-0):
        rest()        
    elif(dir1==1 and dir2==1):
        moveFwd()
    elif(dir1==-1 and dir2==-1):
        moveBkwd()
    elif(dir1==1 and dir2==-1):
        moveR()
    elif(dir1==-1 and dir2==1):
        moveL()
    
def steer_callback(inp):

    global dir1,dir2,speed        
    if(inp.data[0]>-10 and inp.data[0]<10 and inp.data[6]>-10 and inp.data[6]<10):
        speed=0
        dir1=0
        dir2=0
    
    elif(inp.data[0]>10):
        speed=inp.data[0]*inp.data[0]/800
        dir1=-1
        dir2=-1

    elif(inp.data[0]<-10):
        speed=inp.data[0]*inp.data[0]/800
        dir1=1
        dir2=1
        
    elif(inp.data[6]>10):
        speed=inp.data[6]*25*inp.data[6]/800
        dir1=1
        dir2=-1
        
    elif(inp.data[6]<-10):
        speed=inp.data[6]*25*inp.data[6]/800
        dir1=-1
        dir2=1   

def mode_callback(inp):
    global mode
    #print(inp.data)
    if(inp.data):
        mode="steer"
    else:
        mode="drive"
def initial_callback(inp):
        #rospy.loginfo("Subscriber called")
    if roboclaw1 is not None:
            roboclaw1.initialAngleM1 = inp.data[0]*180/3.14
            roboclaw1.initialAngleM2 = inp.data[0]*180/3.14
    if roboclaw2 is not None:
            roboclaw2.initialAngleM1 = inp.data[0]*180/3.14
            roboclaw2.initialAngleM2 = inp.data[0]*180/3.14

def final_callback(inp):

    roboclaw1.targetAngleM1 = inp.data[0]
    roboclaw1.targetAngleM2 = inp.data[0]
    roboclaw2.targetAngleM1 = inp.data[0]
    roboclaw2.targetAngleM2 = inp.data[0]


if __name__ == "__main__":

    rospy.init_node("steer_node")
    rospy.loginfo("Starting steer node")

    #rospy.Subscriber("/ang_msg", Float64MultiArray, final_callback)
    rospy.loginfo("I'm here")

    r_time = rospy.Rate(1)

    for i in range(20):
        try:
            roboclaw1 = SteerClaw(0x80, "/dev/roboclaw1", 9600, "Backward")
        except SerialException:
            rospy.logwarn("Could not connect to RoboClaw2, retrying...")
            r_time.sleep()
    rospy.loginfo("Connected to RoboClaw1")

    
    for i in range(20):
        try:
            roboclaw2 = SteerClaw(0x81, "/dev/roboclaw2", 9600, "forward")
        except SerialException:
            rospy.logwarn("Could not connect to RoboClaw1, retrying...")
            r_time.sleep()
    rospy.loginfo("Connected to RoboClaw2")

    rospy.Subscriber("IMU", Float32MultiArray, initial_callback)
    rospy.Subscriber("/rover/drive_directives", Float64MultiArray, steer_callback)
    rospy.Subscriber("/autonomous_switch",Bool,mode_callback)
    r_time = rospy.Rate(5)
    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)
    roboclaw2.claw.ForwardM1(0)
    roboclaw2.claw.ForwardM2(0)

    roboclaw1.targetAngleM1 = 0
    roboclaw1.targetAngleM2 = 0
    roboclaw2.targetAngleM1 = 0
    roboclaw2.targetAngleM2 = 0


    while not rospy.is_shutdown():
        #print(mode)
        if(mode=="steer"):
            roboclaw1.update()
            roboclaw2.update()
        else:
            update_steer()
        r_time.sleep()

    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)
    roboclaw2.claw.ForwardM1(0)
    roboclaw2.claw.ForwardM2(0)
