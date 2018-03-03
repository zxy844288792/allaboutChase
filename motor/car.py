from distance import *
from port import *
from motor import *
StopThresh = 12
previous = 0

def runCar(left,right,previous):
	distance = GetFrontDist2()
	print(distance)
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
	return previous
	
#GPIO.cleanup()

if __name__ == "__main__":
	(left,right) = port_init()
	previous = 0
	while(1):
		previous = runCar(left,right,previous)

	
