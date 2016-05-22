#!/usr/bin/env python

import numpy as np
import cv2
from collections import Counter

import cPickle
import os

# Define paths for calibration/image files
PATH = os.path.expanduser("~") + '/Sentinel/camera_calibration/OpenCVCalibration/'
CALIB_FILENAME_L = PATH + 'calib/camera_calib_data_L.txt'
CALIB_FILENAME_R = PATH + 'calib/camera_calib_data_R.txt'
IMAGES_PATH = 'test/'
IMAGE_FILE = 'img'
TEST_IMAGE_L =  PATH + IMAGES_PATH + IMAGE_FILE + 'L_1.png'
TEST_IMAGE_R =  PATH + IMAGES_PATH + IMAGE_FILE + 'R_1.png'

# Define wait time for images
IMSHOW_WAIT_TIME = 3000 # 3 seconds

def load_calibration(filename):
	'''
		Loads calibration matrices from given file using cPickle
		# TODO: Use JSON files instead
	'''
	with open(filename, "r") as f:
		mtx, dist, newcameramtx = cPickle.load(f)

	return mtx, dist, newcameramtx

def display_image(image, frame_name="Image"):
    '''
        Displays given image in a named OpenCV window
    '''
    cv2.imshow(frame_name, image)
    cv2.moveWindow(frame_name, 30, 30)
    cv2.waitKey(IMSHOW_WAIT_TIME)
    cv2.destroyAllWindows()

def display_double_image(image_L, image_R, frame_name="Image"):
	'''
		Displays two images (mono) in one OpenCV window
	'''

	try:
		new_img = np.hstack((image_L, image_R))
	except ValueError:
		print("Cannot diplay: dimmension mismatch in pictures")
		return

	# Show snapshot
	display_image(new_img, frame_name)

raw_img_L = cv2.imread(TEST_IMAGE_L, cv2.CV_LOAD_IMAGE_GRAYSCALE)
raw_img_R = cv2.imread(TEST_IMAGE_R, cv2.CV_LOAD_IMAGE_GRAYSCALE)

# Load cailbration data
mtx_l, dist_l, newcameramtx_l = load_calibration(CALIB_FILENAME_L)
mtx_r, dist_r, newcameramtx_r = load_calibration(CALIB_FILENAME_R)

# Use OpenCV function to undistort images
img_l_undistorted = cv2.undistort(raw_img_L, mtx_l, dist_l, None, newcameramtx_l)
img_r_undistorted = cv2.undistort(raw_img_R, mtx_r, dist_r, None, newcameramtx_r)

# Define parameters for disparity map calculation
disparities = [32, 48, 64]
SADWindowSize = [5, 7]
uniquenessRatio = [1]
speckleWindowSize = [250]
speckleRange = [5, 10, 15]

for sad in SADWindowSize:
	for sr in speckleRange:
		for sw in speckleWindowSize:
			for ur in uniquenessRatio:
				for dispa in disparities:

					window_size = 3
					min_disp = 16
					stereo = cv2.StereoSGBM(minDisparity=min_disp,
					numDisparities=dispa,
					SADWindowSize=sad,
					P1 = 8*3*sad*sad,
					P2 = 32*3*sad*sad,
					disp12MaxDiff = 20,
					preFilterCap=32,
					uniquenessRatio = ur,
					speckleWindowSize = sw,
					speckleRange = sr,
					fullDP=True
					)

					disp = stereo.compute(img_r_undistorted, img_l_undistorted).astype(np.float32) / 16.0
					disparity_img = (disp-min_disp)/dispa

					disparity_img = (stereo.compute(img_r_undistorted, img_l_undistorted) / 16.0).astype(np.uint8)

					# Display tuning params in image window
					frame_name = "numDisp=%s, SAD=%s, uniqRat=%s, spekWin=%s, spekRang=%s" % (dispa, sad, ur, sw, sr)
					display_double_image(raw_img_L, disparity_img, frame_name=frame_name)