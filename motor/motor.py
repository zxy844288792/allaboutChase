from port import *
import time
import math

def moveForward(left,right,speed):
	GPIO.output(IN1, True)
	GPIO.output(IN2, False)
	GPIO.output(IN3, False)
	GPIO.output(IN4, True)
	left.start(speed)
	right.start(speed)


def moveStop(left,right):
	left.start(0)
	right.start(0)

def moveBackward(left,right,speed):
	GPIO.output(IN1, False)
	GPIO.output(IN2, True)
	GPIO.output(IN3, True)
	GPIO.output(IN4, False)
	left.start(speed)
	right.start(speed)

def moveLeft(left,right,speed,angle):
	right.start(speed)
	print(int(math.cos(math.radians(angle)) * speed))
	left.start(int(math.cos(math.radians(angle)) * speed))

def moveRight(left,right,speed,angle):
	right.start(math.cos(math.radians(angle)) * speed)
	left.start(speed)
