import time
from port import *

def GetFrontDist_side():
	thresh = 1000
	counter = 0
	pulse_start = 0
	pulse_end = 0
	GPIO.output(TRIG2,False)
	time.sleep(0.006)
	GPIO.output(TRIG2,True)
	time.sleep(0.00001)
	GPIO.output(TRIG2,False)
	pulse_start = time.time()
	while GPIO.input(ECHO2) == 0:
		if(counter < 1000):
			pulse_start = time.time()
			counter += 1
		else:
			return 100
	while GPIO.input(ECHO2) == 1:
	    pulse_end = time.time()
	    
	    
	
	pulse_duration = pulse_end - pulse_start
	distance = pulse_duration * 17150 
	#print("distance: ",distance)
	return distance

def GetFrontDist2():
	thresh = 1000
	counter = 0
	pulse_start = 0
	pulse_end = 0
	GPIO.output(TRIG,False)
	time.sleep(0.006)
	GPIO.output(TRIG,True)
	time.sleep(0.00001)
	GPIO.output(TRIG,False)
	pulse_start = time.time()
	while GPIO.input(ECHO) == 0:
		if(counter < 1000):
			pulse_start = time.time()
			counter += 1
		else:
			return 100
	while GPIO.input(ECHO) == 1:
	    pulse_end = time.time()
	    
	    
	
	pulse_duration = pulse_end - pulse_start
	distance = pulse_duration * 17150 
	#print("distance: ",distance)
	return distance


