####################################################
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
from distance import *
from port import *
from motor import *
from car import runCar
DEBUG = False

def load_calibrations(use_webcam = True):
    calibrationFile = "calibration.yaml"
    if use_webcam:
        calibrationFile = "webcam_calibration.yaml"
    
    with open(calibrationFile) as f:
        loadeddict = yaml.load(f)
    camera_matrix = np.array(loadeddict.get('camera_matrix'))
    dist_coeffs = np.array(loadeddict.get('dist_coeff'))
    print("loaded camera and dist coeff.")
    return camera_matrix, dist_coeffs

def setup_webcam(res, fps):
    cap = cv2.VideoCapture(0)
    cap.set(3, res[0])
    cap.set(4, res[1])
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap

def setup_picam(res, fps):
    camera = PiCamera()
    camera.resolution = res
    camera.framerate = fps
    rawCapture = PiRGBArray(camera, size=res)
    time.sleep(0.1)
    return camera, rawCapture
    
def butter(raw_buffer, pred_buffer, buffer_size, buffer_start):
    #b = np.array([0.2929,0.5858,0.2929])
    #a = np.array([1,0,0.1716])
    #b = np.array([0.0413, 0.0825, 0.0413])
    #a = np.array([1, -1.349, 0.514])
    b = 0.333333333* np.ones(3)
    a = np.array([1, 0, 0])
    new_pred = 0

    for i in range(buffer_size):
        buffer_i = (buffer_start+ buffer_size - i)%buffer_size # first in the buffer
        buffer_i_minus_one = (buffer_start + buffer_size - i - 1)%buffer_size # first -1 in the buffer
        if i  == 0:
            new_pred += b[i]*raw_buffer[buffer_i]
        else:
            new_pred += b[i]*raw_buffer[buffer_i] - a[i] * pred_buffer[buffer_i_minus_one]
    #y_new = b[0]*raw_buff[0] + b[1]*raw_buff[1] + b[2]*raw_buff[2] - a[1]*y_buff[0] - a[2]*y_buff[1]];
    #y_buff.append(y_new()); drop y_buff tail
    pred_buffer[buffer_start] = new_pred
    return new_pred, pred_buffer

def update_buffer(state, raw_buffer, buffer_start, buffer_size):
    if (len(raw_buffer)< buffer_size):
        raw_buffer.append(state)
    else:
        raw_buffer[buffer_start%buffer_size] = state
    buffer_start +=1
    return raw_buffer, buffer_start


def avg_filter(translation, raw_buffer, pred_buffer, buffer_size, buffer_start):
    if len(raw_buffer) < buffer_size:
        raw_buffer.append(translation)
        pred_buffer.append(translation)
        pred = translation
    else:
        buffer_start = buffer_start%buffer_size
        raw_buffer[buffer_start] = translation
        pred, pred_buffer = butter(raw_buffer, pred_buffer, buffer_size, buffer_start)
    
    buffer_start += 1

    return pred, raw_buffer, pred_buffer, buffer_start

def med_filter(u_distance, raw_buffer, buffer_start, buffer_size):
    raw_buffer, buffer_start= update_buffer(u_distance, raw_buffer, buffer_start, buffer_size)
    return np.median(raw_buffer), raw_buffer, buffer_start


if __name__ == "__main__":    
    MARKER_DICT = aruco.DICT_4X4_50
    PRINT_EDGE = 7
    #, new marker size
    BUFFER_FILTER = 3
    RUNNING_BUFFER_SIZE = 5
    TRACKING_BUFFER_SIZE = 10
    AUTO_COUNTDOWN = 2*30
    TRACKING_IDS = [0, 1, 2, 3] # 0, front, 1 left, 2 back, 3 right.
    USE_WEBCAM = False
    TARGET_Z = 15
    TARGET_Z_TOLERANCE = 3
    P_Z = 0.5
    TARGET_U = 10
    TARGET_U_TOLERANCE = 3
    
    START_MOTOR_THRESHOLD = 60
    MAX_SPEED = 100
    RESOLUTION = (320, 240)
    FPS = 30
    
    StopThresh = 6
    (left, right, running, old) = port_init()

    # setup buffers
    pred_buffer_a = []
    raw_buffer_a = []
    buffer_start_a = 0
    started = 0
    
    raw_buffer_u = []
    buffer_start_u = 0
    
    running_buffer = [] # define 0 as stop, 1 as running forward
    running_buffer_start = 0

    tracking_buffer = [] # tracking is 1, lost is 0
    tracking_buffer_start = 0
 
    auto_countdown_counter = 0
    # load calibrations
    camera_matrix, dist_coeffs = load_calibrations(USE_WEBCAM)
    # get aruco info
    aruco_dict = aruco.Dictionary_get(MARKER_DICT)
    markerLength =  PRINT_EDGE  # Here, our measurement unit is centimetre.
    arucoParams = aruco.DetectorParameters_create()
    # setup camera
    if USE_WEBCAM:
        cap = setup_webcam(RESOLUTION, FPS)
        while(True):
            ret, img = cap.read()
            if ret == True:
                img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    # aruco.detectMarkers() requires gray image
                corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, aruco_dict, parameters=arucoParams) # Detect aruco
                if np.any(ids != None): # if aruco marker detected
                    found = -1
                    for i, id in enumerate(ids[0]):
                        if id < 4:
                            # tracking IDs
                            found = i
                            print("found %d"%found)
                        else:
                            print("bigger or equal to 4")
                    if found != -1:
                        tracked_id = ids[0][found]
                        rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # posture estimation from a single marker
                        
                        translation = np.array(tvec[0][found])
                        #print("r: %.8f %.8f %.8f"%(translation[0], translation[1], translation[2]))
                        # treshold the values within 10cm distanc change.
                        if started ==0 :
                            started =1
                            prevs = translation
                        else:
                            if np.linalg.norm(translation-prevs) > 10:
                                translation = prevs
                            else:
                                prevs = translation

                        # use butter buffer
                        translation, raw_buffer_a, pred_buffer_a, buffer_start_a = avg_filter(translation, raw_buffer_a, pred_buffer_a, BUFFER_FILTER, buffer_start_a)
                        
                        #print("p: %.8f %.8f %.8f"%(translation[0], translation[1], translation[2]))
                        print("distance:%.2f"%translation[2])
                        if DEBUG:
                            imgWithAruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
                        #imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 100)    # axis length 100 can be changed according to your requirement
                    else:
                        if DEBUG:
                            imgWithAruco = img
                else:   # if aruco marker is NOT detected
                    print("tracking lost")
                    if DEBUG:
                        imgWithAruco = img  # assign imRemapped_color to imgWithAruco directly
        
                if DEBUG:
                    cv2.imshow("aruco", imgWithAruco)   # display
        
                if cv2.waitKey(2) & 0xFF == ord('q'):   # if 'q' is pressed, quit.
                    break
    else:
        camera, rawCapture = setup_picam(RESOLUTION, FPS)
        u_distance_0 = 0
        print("set up pi cam")
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            #print("got a frame")
            tracking(False)
            lost(False)
            u_tracking(False)
            if running:
                u_distance_0 = GetFrontDist2()
                #u_distance_0, raw_buffer_u, buffer_start_u = med_filter(u_distance_0, raw_buffer_u, buffer_start_u, BUFFER_FILTER)
                print("u: %.8f"%u_distance_0)
                if u_distance_0 < 7:
                        
                    error_u = u_distance_0 - TARGET_U
                    if error_u < - TARGET_U_TOLERANCE:
                        u_tracking(True)
                        lost(True) 
                        speed = START_MOTOR_THRESHOLD
                        moveBackward(left, right, speed)
                        if DEBUG:
                            print("moving back, u_sensor:", u_distance_0)
                    elif error_u > TARGET_U_TOLERANCE:
                        if DEBUG:
                            print("Warning: trusting u sensor with error_u %d"%error_u)
                    # when we are in close distance, we update states as tracking lost, and parked while we adjust based on distance sensor.
                    tracking_buffer, tracking_buffer_start = update_buffer(0, tracking_buffer, tracking_buffer_start, TRACKING_BUFFER_SIZE)
                    running_buffer, running_buffer_start = update_buffer(0, running_buffer, running_buffer_start, RUNNING_BUFFER_SIZE)

                else:
                    #print("streaming frames")
                    img = frame.array
                    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    # aruco.detectMarkers() requires gray image
                    corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, aruco_dict, parameters=arucoParams) # Detect aruco
                    if np.any(ids != None): # if aruco marker detected
                        found = -1
                        for i, id in enumerate(ids[0]):
                            if id < 4:
                                # tracking IDs
                                found = i
                        if found != -1:
                            tracking(True)
                            auto_countdown_counter = AUTO_COUNTDOWN
                            tracked_id = ids[0][found]
                            rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # posture estimation from a single marker
                            
                            translation = np.array(tvec[0][found])
                            #print("r: %.8f %.8f %.8f"%(translation[0], translation[1], translation[2]))
                            # treshold the values within 10cm distanc change.
                            if started ==0 :
                                started =1
                                prevs = translation
                            else:
                                if np.linalg.norm(translation-prevs) > 10:
                                    translation = prevs
                                else:
                                    prevs = translation

                            # use average filter
                            translation, raw_buffer_a, pred_buffer_a, buffer_start_a = avg_filter(translation, raw_buffer_a, pred_buffer_a, BUFFER_FILTER, buffer_start_a)
                            
                            print("p: %.8f %.8f %.8f"%(translation[0], translation[1], translation[2]))
                            #print("aruco", translation)
                            z_distance = translation[2]
                            
                            error_z = z_distance - TARGET_Z
                            forward_speed = P_Z*error_z
                            
                            x_distance = translation[0]
                            error_x = x_distance
                            TARGET_X_TOLERANCE = 2
                            angle = math.atan(abs(x_distance)/abs(z_distance))
                            
                            speed = min(MAX_SPEED, START_MOTOR_THRESHOLD + forward_speed)
                            
                            if (error_z > TARGET_Z_TOLERANCE and error_x <- TARGET_X_TOLERANCE):
                                print ("moving foward left at speed %d to correct error_z=%d and angle %.4f"%(speed, error_z, error_z)) 
                                moveLeft(left,right, START_MOTOR_THRESHOLD , MAX_SPEED)
                            elif (error_z > TARGET_Z_TOLERANCE and error_x >TARGET_X_TOLERANCE):
                                print ("moving foward right at speed %d to correct error_z=%d and angle %.4f"%(speed, error_z, error_z)) 
                                moveRight(left,right, MAX_SPEED , START_MOTOR_THRESHOLD)
                            elif error_z > TARGET_Z_TOLERANCE:
                                print ("moving foward at speed %d to correct error_z=%d and angle %.4f"%(speed, error_z, error_z)) 
                                moveForward(left, right, speed)

                            elif error_z < - TARGET_Z_TOLERANCE:
                                print ("moving backward at speed %d to correct error_z = %d"%(speed, error_z))
                                moveBackward(left, right, speed)
                                # stop
                            else:
                                print("stop, tag: %d"%(tracked_id, ))
                                moveStop(left, right)
                               
                            # if we are tracking, we update states as follows
                            tracking_buffer, tracking_buffer_start = update_buffer(1, tracking_buffer, tracking_buffer_start, TRACKING_BUFFER_SIZE)
                            running_state = 0
                            if error_z > TARGET_Z_TOLERANCE:
                                running_state = 1
                            # we are only going to act if the running state is running foward.
                            running_buffer, running_buffer_start = update_buffer(running_state, running_buffer, running_buffer_start, RUNNING_BUFFER_SIZE)

 
                            # comment this out to gain speed:
                            if DEBUG:
                                imgWithAruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
                            #imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 100)    # axis length 100 can be changed according to your requirement
                        else:
                            if DEBUG:
                                imgWithAruco = img
                            print("no markers within defined range is found lost")
                            # if no markers within defined range is found, it's also tracking lost.
                            tracking_buffer, tracking_buffer_start = update_buffer(0, tracking_buffer, tracking_buffer_start, TRACKING_BUFFER_SIZE)
                            #if sum(tracking_buffer) < TRACKING_BUFFER_SIZE/2:
                            # tracking truly lost
                            if auto_countdown_counter <= 0:
                                moveStop(left, right)
                            else:
                                auto_countdown_counter -= 1
                                # autopilot
                                if sum(running_buffer) > RUNNING_BUFFER_SIZE/2:
                                    moveForward(left, right, START_MOTOR_THRESHOLD)
                                    running_buffer, running_buffer_start = update_buffer(1, running_buffer, running_buffer_start, RUNNING_BUFFER_SIZE)
                                else:
                                    moveStop(left, right)
                                    running_buffer, running_buffer_start = update_buffer(0, running_buffer, running_buffer_start, RUNNING_BUFFER_SIZE)

                    else:   # if aruco marker is NOT detected
                        if DEBUG:
                            imgWithAruco = img  # assign imRemapped_color to imgWithAruco directly
                        if auto_countdown_counter <= 0:
                            moveStop(left, right)
                        else:
                            auto_countdown_counter -= 1
                            # autopilot
                            if sum(running_buffer) > RUNNING_BUFFER_SIZE/2:
                                print("Auto forward")
                                moveForward(left, right, START_MOTOR_THRESHOLD)
                                running_buffer, running_buffer_start = update_buffer(1, running_buffer, running_buffer_start, RUNNING_BUFFER_SIZE)
                            else:
                                moveStop(left, right)
                                running_buffer, running_buffer_start = update_buffer(0, running_buffer, running_buffer_start, RUNNING_BUFFER_SIZE)

                        print("tracking lost")
                        lost(True)
                    if DEBUG:
                        cv2.imshow("aruco", imgWithAruco)   # display
        
            else:
                moveStop(left, right)
                running_buffer = []
                running_buffer_start = 0
                tracking_buffer =[]
                tracking_buffer_start = 0
                pred_buffer_a =[]
                raw_buffer_a =[]
                raw_buffer_u = []
                pred_buffer_a_start = 0
                raw_buffer_a_start =0
                raw_buffer_u_start = 0
                auto_countdown_counter = 0
            rawCapture.truncate(0)

            if old == 1 and GPIO.input(BUTTON) == 0:
                running = 1 - running
                print("Running statue: %d"%running)
                
            old = GPIO.input(BUTTON)
            
            if cv2.waitKey(2) & 0xFF == ord('q'):   # if 'q' is pressed, quit.
                break
 
