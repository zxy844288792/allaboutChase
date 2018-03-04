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

def port_init():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(TRIG, GPIO.OUT)
	GPIO.setup(ECHO, GPIO.IN)
	GPIO.setup(EN1, GPIO.OUT)
	GPIO.setup(IN1, GPIO.OUT)
	GPIO.setup(IN2, GPIO.OUT)
	GPIO.setup(EN2, GPIO.OUT)
	GPIO.setup(IN3, GPIO.OUT)
	GPIO.setup(IN4, GPIO.OUT)
	GPIO.setup(BUTTON, GPIO.IN)

	GPIO.output(TRIG, False)
	GPIO.output(IN1, True)
	GPIO.output(IN2, False)
	GPIO.output(IN3, True)
	GPIO.output(IN4, False)
	left = GPIO.PWM(EN1,100)
	left.start(0)
	right = GPIO.PWM(EN2,100)
	right.start(0)
	running = 1
	old = 1
	return (left,right,running,old)
	
	