################################################################################
#                                                                              #
#                                                                              #
#           IMPORTANT: READ BEFORE DOWNLOADING, COPYING AND USING.             #
#                                                                              #
#                                                                              #
#      Copyright [2017] [ShenZhen Longer Vision Technology], Licensed under    #
#      ******** GNU General Public License, version 3.0 (GPL-3.0) ********     #
#      You are allowed to use this file, modify it, redistribute it, etc.      #
#      You are NOT allowed to use this file WITHOUT keeping the License.       #
#                                                                              #
#      Longer Vision Technology is a startup located in Chinese Silicon Valley #
#      NanShan, ShenZhen, China, (http://www.longervision.cn), which provides  #
#      the total solution to the area of Machine Vision & Computer Vision.     #
#      The founder Mr. Pei JIA has been advocating Open Source Software (OSS)  #
#      for over 12 years ever since he started his PhD's research in England.  #
#                                                                              #
#      Longer Vision Blog is Longer Vision Technology's blog hosted on github  #
#      (http://longervision.github.io). Besides the published articles, a lot  #
#      more source code can be found at the organization's source code pool:   #
#      (https://github.com/LongerVision/OpenCV_Examples).                      #
#                                                                              #
#      For those who are interested in our blogs and source code, please do    #
#      NOT hesitate to comment on our blogs. Whenever you find any issue,      #
#      please do NOT hesitate to fire an issue on github. We'll try to reply   #
#      promptly.                                                               #
#                                                                              #
#                                                                              #
# Version:          0.0.1                                                      #
# Author:           JIA Pei                                                    #
# Contact:          jiapei@longervision.com                                    #
# URL:              http://www.longervision.cn                                 #
# Create Date:      2017-03-10                                                 #
################################################################################

import sys
sys.path.append('/usr/local/python/3.5')  # whichever folder that contains **cv2.so** when you built OpenCV

import os
import cv2
from cv2 import aruco
import numpy as np
import yaml
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
#from scipy.stats import mode
from collections import Counter
#from statistics import mode
from distance import *
from port import *
from motor import *
from car import runCar

StopThresh = 6
(left, right) =  port_init()

RESOLUTION = (320, 240)
FRAMERATE = 30
MARKER_DICT = aruco.DICT_4X4_50

calibrationFile = "calibration.yaml"
with open(calibrationFile) as f:
    loadeddict = yaml.load(f)
camera_matrix = np.array(loadeddict.get('camera_matrix'))
dist_coeffs = np.array(loadeddict.get('dist_coeff'))
print(camera_matrix)
print(type(camera_matrix))
print(dist_coeffs)
#r = calibrationParams.getNode("R").mat()
#new_camera_matrix = calibrationParams.getNode("newCameraMatrix").mat()

#image_size = (1920, 1080)
#map1, map2 = cv2.fisheye.initUndistortRectifyMap(camera_matrix, dist_coeffs, r, new_camera_matrix, image_size, cv2.CV_16SC2)



aruco_dict = aruco.Dictionary_get(MARKER_DICT)
markerLength = 5   # Here, our measurement unit is centimetre.
arucoParams = aruco.DetectorParameters_create()

# to use webcam, comment out the following 5 lines.
camera = PiCamera()
camera.resolution = RESOLUTION
camera.framerate = FRAMERATE
rawCapture = PiRGBArray(camera, size=RESOLUTION)
time.sleep(0.1)
# to use webcam, uncomment this line:
circular_buffer_states = [] # keep track of which side of the car is presented.[0, 1, 2, 3] means [back, left, front, right]
circular_buffer_z = [] # keep track of distance. [1, 0, -1] meaning forward, parked, and backwards
circular_buffer_z_speed = [] # keep track of relative speed.
circular_buffer_lost = []#[0, 1] if 0: lost, if 1 tracking.
last_location = 0
circular_buffer_size = 10
start_buffer =0
start_buffer_lost =0
prev_z = 0
# cap = cv2.VideoCapture(0)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
    # to use webcam, uncomment the following    
    #while(True):
    #    ret, img = cap.read()
    imgRemapped = img
    #if ret == True:
    #imgRemapped = cv2.remap(img, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT) # for fisheye remapping
    imgRemapped_gray = cv2.cvtColor(imgRemapped, cv2.COLOR_BGR2GRAY)    # aruco.detectMarkers() requires gray image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(imgRemapped_gray, aruco_dict, parameters=arucoParams) # Detect aruco
    if np.any(ids != None): # if aruco marker detected
        print(ids)
        first = ids[0][0]
        #for i in range(len(ids)):
            #print("id %d"%ids[i])
            #print(corners[i][0][0])
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # posture estimation from a single marker
        print(tvec[0][0][0])
        # populates the circular buffer
        speed = prev_z - tvec[0][0][0]
        print("speed")
        print(speed)
        prev_z = tvec[0][0][0]
        z_state = 0;
        if speed > 0:
            z_state = 1
        elif speed <0:
            z_state = -1
   
        if (len(circular_buffer_states) == circular_buffer_size):
            idx = start_buffer%circular_buffer_size
            circular_buffer_states[idx] = first
            circular_buffer_z_speed[idx] = speed
            circular_buffer_z[idx] = z_state
        else:
            circular_buffer_states.append(first)
            circular_buffer_z_speed.append(speed)
            circular_buffer_z.append(z_state)
        start_buffer +=1
        
        if (len(circular_buffer_lost) == circular_buffer_size):
            idx = start_buffer_lost%circular_buffer_size
            circular_buffer_lost[idx] = 1
        else:
            circular_buffer_lost.append(1)

        # get the mode of the car.
        #print (circular_buffer_states)
        status = Counter(circular_buffer_states).most_common(1)[0][0]
        forward_status = np.sum(circular_buffer_z)
        #if GetFrontDist2() <=  StopThresh and forward_status >= 0:
        #    print("park since too close")
        #    moveStop(left,right)
        #else:
	distance = GetFrontDist2()
	print("front dist ",distance)
	if status == 0 and forward_status >0:
	    print("go straight")
	    moveForward(left, right, 100)
	elif status == 0 and forward_status == 0:
	    print ("park")
	    moveStop(left,right)
	elif status ==0 and forward_status <0:
	    print("back up")
	    moveBackward(left, right, 100)
	elif status == 1:
	    print ("turn left")
	    moveLeft(left, right,100, 45)
	elif status == 2:
	    print ("park and get ready to turn around")
	    moveStop(left, right)
	elif status == 3:
	    print ("turn right")
            moveRight(left, right, 100, 45)
        
        #imgWithAruco = aruco.drawDetectedMarkers(imgRemapped, corners, ids, (0,255,0))
        #imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 50)    # axis length 100 can be changed according to your requirement
    else:   # if aruco marker is NOT detected
        moveStop(left, right)
        imgWithAruco = imgRemapped  # assign imRemapped_color to imgWithAruco directly
        if (len(circular_buffer_lost) == circular_buffer_size):
            idx = start_buffer_lost%circular_buffer_size
            circular_buffer_lost[idx] = 0
        else:
            circular_buffer_lost.append(0)
    lost=np.sum(circular_buffer_lost)
    if lost == 0:
        print("signal lost")
    start_buffer_lost +=1
    #cv2.imshow("aruco", imgWithAruco)   # display
    
    
    if cv2.waitKey(2) & 0xFF == ord('q'):   # if 'q' is pressed, quit.
        break
    rawCapture.truncate(0)
