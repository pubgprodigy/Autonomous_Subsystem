import RPi.GPIO as GPIO # import GPIO library
from time import sleep

GPIO.cleanup()

Motor1A = 16 # set GPIO-02 as Input 1 of the controller IC
Motor1B = 12 # set GPIO-03 as Input 2 of the controller IC
Motor2A = 26 # set GPIO-02 as Input 1 of the controller IC
Motor2B = 19 # set GPIO-03 as Input 2 of the controller IC

GPIO.setmode(GPIO.BCM)
GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor2A,GPIO.OUT)
GPIO.setup(Motor2B,GPIO.OUT)

def rotate_acw():
	GPIO.output(Motor1A,GPIO.HIGH)
	GPIO.output(Motor1B,GPIO.LOW)
	GPIO.output(Motor2A,GPIO.HIGH)
	GPIO.output(Motor2B,GPIO.LOW)
#	sleep(time)
#	GPIO.cleanup()

def rotate_cw():
	GPIO.output(Motor1A,GPIO.LOW)
	GPIO.output(Motor1B,GPIO.HIGH)
	GPIO.output(Motor2A,GPIO.LOW)
	GPIO.output(Motor2B,GPIO.HIGH)
#	sleep(time)
#	GPIO.cleanup()

def move_forward():
	GPIO.output(Motor1A,GPIO.HIGH)
	GPIO.output(Motor1B,GPIO.LOW)
	GPIO.output(Motor2A,GPIO.LOW)
	GPIO.output(Motor2B,GPIO.HIGH)
#	sleep(time)
#	GPIO.cleanup()

def move_back():
	GPIO.output(Motor1A,GPIO.LOW)
	GPIO.output(Motor1B,GPIO.HIGH)
	GPIO.output(Motor2A,GPIO.HIGH)
	GPIO.output(Motor2B,GPIO.LOW)
#	sleep(time)
#	GPIO.cleanup()

def halt():
	GPIO.output(Motor1A,GPIO.LOW)
        GPIO.output(Motor1B,GPIO.LOW)
        GPIO.output(Motor2A,GPIO.LOW)
        GPIO.output(Motor2B,GPIO.LOW)

def stop():
	GPIO.cleanup()
