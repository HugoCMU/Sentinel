#!/usr/bin/env python
# 03/14/2016 HugoCMU
# Program will get calibration matrices for cameras

# NOTE* To calibrate the caamera using ROS:
# rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.023 right:=/stereo/cv_camera_right_node/image_raw left:=/stereo/cv_camera_left_node/image_raw right_camera:=/stereo/cv_camera_right_node/ left_camera:=/stereo/cv_camera_left_node/


import numpy as np
import cv2
import glob
import os
import cPickle

# Define absolute path for calibration files
PATH = os.path.expanduser("~") + '/Sentinel/camera_calibration/OpenCVCalibration/'

# Name variables for files
# IMAGES_PATH = 'tmp/'
IMAGES_PATH = 'hist/22_05_2016__00_16/'
CALIB_PATH = 'calib/camera_calib_data_'
IMAGE_FILE = 'img'
CAMERA_NAME = 'L'
# CAMERA_NAME = 'R'
TEST_IMAGE =  PATH + IMAGES_PATH + IMAGE_FILE + CAMERA_NAME + '_2.png'

# Size of Checkerboard
CHECK_SIZE = (9, 7) # 9 by 7 board
CHECK_DIMM = 23 # 23 mm

# Define wait time for images
IMSHOW_WAIT_TIME = 3000 # 3 seconds

def display_image(image, frame_name="Image"):
    '''
        Displays given image in a named OpenCV window
    '''
    cv2.imshow(frame_name, image)
    cv2.moveWindow(frame_name, 30, 30) # Coordinate (X,Y) = (30,30)
    cv2.waitKey(IMSHOW_WAIT_TIME)
    cv2.destroyAllWindows()

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, CHECK_DIMM, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((CHECK_SIZE[1]*CHECK_SIZE[0],3), np.float32)
objp[:,:2] = np.mgrid[0:CHECK_SIZE[0],0:CHECK_SIZE[1]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Take images from tmp directory
images = glob.glob(PATH + IMAGES_PATH + IMAGE_FILE + CAMERA_NAME + '_*.png')

for i, fname in enumerate(images):
    gray = cv2.imread(fname, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    color = cv2.cvtColor(gray,cv2.COLOR_GRAY2RGB)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (CHECK_SIZE[0],CHECK_SIZE[1]), None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(color, (CHECK_SIZE[0],CHECK_SIZE[1]), corners, ret)
        display_image(color, frame_name="Frame %s" % i)


# returns the camera matrix, distortion coefficients, rotation and translation vectors
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

img = cv2.imread(TEST_IMAGE)
h,w,_ = img.shape
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

display_image(img, frame_name="Uncalibrated")

# Write calibration matrices into pickle file
with open(PATH + CALIB_PATH + CAMERA_NAME + '.txt', "w") as f:
    cPickle.dump((mtx, dist, newcameramtx), f)

# Undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

display_image(dst, frame_name="Calibrated")

# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]

display_image(dst, frame_name="Calibrated and Cropped")

# Determine effectiveness of calibration
tot_error = 0
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

print "total error: ", tot_error/len(objpoints)