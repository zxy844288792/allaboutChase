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
# Create Date:      2017-03-19                                                 #
################################################################################

import numpy as np
import cv2
import yaml
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

RESOLUTION = (320, 240)
FRAMERATE = 30
crow=8
ccol=6
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((crow*ccol,3), np.float32)
objp[:,:2] = np.mgrid[0:ccol,0:crow].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
    
# use pi camera: comment out following 5 lines if using webcam
camera = PiCamera()
camera.resolution = RESOLUTION
camera.framerate = FRAMERATE
rawCapture = PiRGBArray(camera, size=RESOLUTION)
time.sleep(0.1) # allow camera to warm up

# use webcam: uncomment this line
#cap = cv2.VideoCapture(0)
found = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    if (found >= 20):  # Here, 10 can be changed to whatever number you like to choose
        break;
    # using pi camera, to use webcam, comment out the following two lines
    img = frame.array
    # to use webcam, uncomment the following line.
    # ret, img = cap.read() # Capture frame-by-frame
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (ccol,crow), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)  # Certainly, every loop objp is the same, in 3D.

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (ccol,crow), corners2, ret)
        found += 1
        print("found %d chessboards"%found)
    else:
        print("no corners found")
    cv2.imshow('img', img)
    cv2.waitKey(1000)
    rawCapture.truncate(0)


# When everything done, release the capture
# to use webcam, uncomment this line. 
# cap.release()
cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


# It's very important to transform the matrix to list.
data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}

with open("calibration.yaml", "w") as f:
    yaml.dump(data, f)


# You can use the following 4 lines of code to load the data in file "calibration.yaml"
# with open('calibration.yaml') as f:
#     loadeddict = yaml.load(f)
# mtxloaded = loadeddict.get('camera_matrix')
# distloaded = loadeddict.get('dist_coeff')

