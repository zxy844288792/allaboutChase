from distance import *
from port import *
from motor import *
StopThresh = 12
previous = 0


def runCar(left,right,previous,running,old):
	if running == 1:
		distance = GetFrontDist2()
		print(distance)
	else:
		pass
	if old == 1 and GPIO.input(BUTTON) == 0:
		running = 1 - running
	old = GPIO.input(BUTTON)

	'''
	if distance > StopThresh:
		if previous == 0:
			moveForward(left,right,100)
			previous = 1
		else:
			moveLeft(left,right,100,70)
			print("move left")
	else:
		previous = 0
		moveStop(left,right)
	'''
	return previous,running,old
	
#GPIO.cleanup()

if __name__ == "__main__":
	(left,right,running,old) = port_init()
	previous = 0
	while(1):
		previous,running,old = runCar(left,right,previous,running,old)

	
