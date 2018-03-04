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
from distance import *
from port import *
from motor import *
from car import runCar

def load_calibrations(use_webcam = True):
    calibrationFile = "calibration.yaml"
    if use_webcam:
        calibrationFile = "webcam_calibration.yaml"
    
    with open(calibrationFile) as f:
        loadeddict = yaml.load(f)
    camera_matrix = np.array(loadeddict.get('camera_matrix'))
    dist_coeffs = np.array(loadeddict.get('dist_coeff'))
    print "loaded camera and dist coeff."
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
    return rawCapture

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

def update_buffers(translation, raw_buffer, pred_buffer, buffer_size, buffer_start):
    if len(raw_buffer) < buffer_size:
        raw_buffer.append(translation)
        pred_buffer.append(translation)
        buffer_start += 1
        pred = translation
    else:
        buffer_start = buffer_start%buffer_size
        raw_buffer[buffer_start] = translation
        pred, pred_buffer = butter(raw_buffer, pred_buffer, buffer_size, buffer_start)
        buffer_start += 1

    return pred, raw_buffer, pred_buffer, buffer_start

if __name__ == "__main__":    
    MARKER_DICT = aruco.DICT_4X4_50
    PRINT_EDGE = 5#7.7, new marker size
    BUFFER_SIZE = 3
    TRACKING_IDS = [0, 1, 2, 3] # 0, front, 1 left, 2 back, 3 right.
    USE_WEBCAM = True
    ERROR = 10
    RESOLUTION = (320, 240)
    FPS
    StopThresh = 6
    (left, right) = port_init()

    # setup buffers
    pred_buffer = []
    raw_buffer = []
    buffer_start = 0
    started = 0
    
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
                        translation, raw_buffer, pred_buffer, buffer_start = update_buffers(translation, raw_buffer, pred_buffer, BUFFER_SIZE, buffer_start)
                        
                        #print("p: %.8f %.8f %.8f"%(translation[0], translation[1], translation[2]))
                        print("distance:%.2f"%translation[2])
                        imgWithAruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
                        #imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 100)    # axis length 100 can be changed according to your requirement
                    else:
                        imgWithAruco = img
                else:   # if aruco marker is NOT detected
                    imgWithAruco = img  # assign imRemapped_color to imgWithAruco directly
        
                cv2.imshow("aruco", imgWithAruco)   # display
        
                if cv2.waitKey(2) & 0xFF == ord('q'):   # if 'q' is pressed, quit.
                    break
    else:
        cap = setup_picam(RESOLUTION, FPS)
        for frame in camera.capture_capture_continuous(rawCapture, format="bgr", use_video_port=True):
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
                    translation, raw_buffer, pred_buffer, buffer_start = update_buffers(translation, raw_buffer, pred_buffer, BUFFER_SIZE, buffer_start)
                    
                    #print("p: %.8f %.8f %.8f"%(translation[0], translation[1], translation[2]))
                    print("distance:%.2f"%translation[2])
                    z-distance = translation[2]
                    #(TODO) use z-distanc e here.
                    # comment this out to gain speed:
                    imgWithAruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
                    #imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 100)    # axis length 100 can be changed according to your requirement
                else:
                    imgWithAruco = img
            else:   # if aruco marker is NOT detected
                imgWithAruco = img  # assign imRemapped_color to imgWithAruco directly
        
            cv2.imshow("aruco", imgWithAruco)   # display
        
            if cv2.waitKey(2) & 0xFF == ord('q'):   # if 'q' is pressed, quit.
                break
 
