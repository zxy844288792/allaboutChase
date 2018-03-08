import RPi.GPIO as GPIO

TRIG = 23
ECHO = 24
EN1 = 17
IN1 = 27
IN2 = 22

EN2 = 2
IN3 = 4
IN4 = 3
BUTTON = 10
frontL = 26
frontR = 9
backL = 21
backR = 20
RED = 5
GREEN = 6
YELLOW = 13
TRIG2 = 16
ECHO2 = 12


def port_init():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(TRIG, GPIO.OUT)
	GPIO.setup(ECHO, GPIO.IN)
	GPIO.setup(TRIG2, GPIO.OUT)
	GPIO.setup(ECHO2, GPIO.IN)
	GPIO.setup(EN1, GPIO.OUT)
	GPIO.setup(IN1, GPIO.OUT)
	GPIO.setup(IN2, GPIO.OUT)
	GPIO.setup(EN2, GPIO.OUT)
	GPIO.setup(IN3, GPIO.OUT)
	GPIO.setup(IN4, GPIO.OUT)
	GPIO.setup(BUTTON, GPIO.IN)
	GPIO.setup(frontL, GPIO.OUT)
	GPIO.setup(frontR, GPIO.OUT)
	GPIO.setup(backL, GPIO.OUT)
	GPIO.setup(backR, GPIO.OUT)
	GPIO.setup(RED, GPIO.OUT)
	GPIO.setup(GREEN, GPIO.OUT)
	GPIO.setup(YELLOW, GPIO.OUT)

	GPIO.output(TRIG, False)
	GPIO.output(TRIG2, False)
	GPIO.output(IN1, True)
	GPIO.output(IN2, False)
	GPIO.output(IN3, True)
	GPIO.output(IN4, False)
	GPIO.output(frontL, False)
	GPIO.output(frontR, False)
	GPIO.output(backL, False)
	GPIO.output(backR, False)
	GPIO.output(RED, False)
	GPIO.output(GREEN, False)
	GPIO.output(YELLOW, False)
	
	left = GPIO.PWM(EN1,100)
	left.start(0)
	right = GPIO.PWM(EN2,100)
	right.start(0)
	running = 0
	old = 1
	return (left,right,running,old)
	
def tracking(x):
	if(x):
		GPIO.output(GREEN, True)
		GPIO.output(YELLOW, False)
		GPIO.output(RED, False)	
	else:
		GPIO.output(GREEN, False)
		GPIO.output(YELLOW, False)
		GPIO.output(RED, False)


def lost(x):
	if(x):
		GPIO.output(GREEN, False)
		GPIO.output(YELLOW, False)
		GPIO.output(RED, True)	
	else:
		GPIO.output(GREEN, False)
		GPIO.output(YELLOW, False)
		GPIO.output(RED, False)


def u_tracking(x):
	if(x):
		GPIO.output(GREEN, False)
		GPIO.output(YELLOW, True)
		GPIO.output(RED, False)	
	else:
		GPIO.output(GREEN, False)
		GPIO.output(YELLOW, False)
		GPIO.output(RED, False)

	
	
