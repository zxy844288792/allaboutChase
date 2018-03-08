from port import *
import time
import math
SLOW = 45
FAST = 70
def moveForward(left,right,speed):
	GPIO.output(IN1, True)
	GPIO.output(IN2, False)
	GPIO.output(IN3, False)
	GPIO.output(IN4, True)
	left.start(speed)
	right.start(speed)
	
	GPIO.output(frontL,True)
	GPIO.output(frontR,True)
	GPIO.output(backL,False)
	GPIO.output(backR,False)
	


def moveStop(left,right):
	left.start(0)
	right.start(0)
	
	GPIO.output(frontL,False)
	GPIO.output(frontR,False)
	GPIO.output(backL,False)
	GPIO.output(backR,False)
	

def moveBackward(left,right,speed):
	GPIO.output(IN1, False)
	GPIO.output(IN2, True)
	GPIO.output(IN3, True)
	GPIO.output(IN4, False)
	left.start(speed)
	right.start(speed)
	
	GPIO.output(frontL,False)
	GPIO.output(frontR,False)
	GPIO.output(backL,True)
	GPIO.output(backR,True)
	

def moveLeft(left,right,speed_left,speed_right):
	GPIO.output(IN1, True)
	GPIO.output(IN2, False)
	GPIO.output(IN3, False)
	GPIO.output(IN4, True)
	right.start(speed_right)
	#print(int(math.cos(angle) * speed))
	#left.start(int(math.cos(angle) * speed))
	left.start(speed_left)
	GPIO.output(frontL,True)
	GPIO.output(frontR,False)
	GPIO.output(backL,False)
	GPIO.output(backR,False)
	

def moveRight(left,right,speed_left,speed_right):
	GPIO.output(IN1, True)
	GPIO.output(IN2, False)
	GPIO.output(IN3, False)
	GPIO.output(IN4, True)
	#right.start(math.cos(math.radians(angle)) * speed)
	#left.start(speed)
	right.start(speed_right)
	left.start(speed_left)
	GPIO.output(frontL,False)
	GPIO.output(frontR,True)
	GPIO.output(backL,False)
	GPIO.output(backR,False)
