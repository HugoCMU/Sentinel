#!/usr/bin/env python
# 03/14/2016 HugoCMU
# Program will take a snapshot with both cameras

from __future__ import print_function

import cv2
import os
import time

import sys
import getopt

import numpy as np

# ----------------- GET PROPER FILENAMES & PATHS
# Define wait time for images
IMSHOW_WAIT_TIME = 3000 # 3 seconds

# Make sure we use proper directory (tmp folder by default)
os.chdir(os.path.expanduser("~") + '/Sentinel/camera_calibration/OpenCVCalibration/tmp')

# Define name of camera calibration img_R_XXX.png
FILENAME_PREFIX = 'img'

# folder name for current session
foldername = time.strftime("%d_%m_%Y__%H_%M")

# ----------------- TAKE USER INPUT

try:
	# command line arguments
	for arg in getopt.getopt(sys.argv,"op")[1]:
		if arg in ("-o"):
			print("-o argument used, overwriting images in tmp")

			# Remove current pictures in tmp folder
			for f in (f for f in os.listdir(".")):
				os.remove(f)

		elif arg in ("-p"): # TODO: Allow for long option that defines folder name
			print("-p argument used, creating new folder in hist")

			# Exit the tmp folder
			os.chdir('../hist')

			# Create/enter date folder if it does not exist
			if not os.path.exists(foldername):
				os.makedirs(foldername)
				os.chdir(foldername)

except getopt.GetoptError:
	print("\n")
	print("-o 	overwrite current generated images in tmp")
	print("-p  push generated images into new date folder")
	print("\n")
	sys.exit(2)

# --------------------------- CAPTURE IMAGES

def rotate_imgs(raw_image_right, raw_image_left):
	'''
		Rotates left and right images to proper orientation. Note this is done with transpose and flip
		as opposed to cv2.warpAffine to improve speed.
	''' 

	# Rotate left image
	image_left = cv2.transpose(raw_image_left)
	image_left = cv2.flip(image_left, 0)

	# Rotate right image
	image_right = cv2.transpose(raw_image_right)
	image_right = cv2.flip(image_right, 1)

	return image_right, image_left

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

# Create camera capture objects
cam_left = cv2.VideoCapture(1)
cam_right = cv2.VideoCapture(0)

# Counter for number of snapshots taken
snapshot_iter = 0

# Keep taking pictures until user exits the program
while True:

	# # Give user time to move board
	# for i in xrange(3):
	# 	print("Countdown: %s....." % (3 - i))
	# 	time.sleep(1)

	# Grab image directly from cv2 video capture object
	# TODO: Put camera reading in thread, so latest image is easier to extract from image buffer
	# http://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/
	for _ in range(5): 
		foo, cv_image_right = cam_right.read()
		foo, cv_image_left = cam_left.read()

	# Convert color images to gray for calibration
	image_right_mono = cv2.cvtColor(cv_image_right, cv2.COLOR_BGR2GRAY)
	image_left_mono = cv2.cvtColor(cv_image_left, cv2.COLOR_BGR2GRAY)

	# Rotate images
	image_right, image_left = rotate_imgs(image_right_mono, image_left_mono)

	# Save images to file
	cv2.imwrite(FILENAME_PREFIX + 'R_%s.png' % snapshot_iter, image_right)
	cv2.imwrite(FILENAME_PREFIX + 'L_%s.png' % snapshot_iter, image_left)

	# Increase image counter
	snapshot_iter += 1

	# Visual confirmation
	print("Snapshot #%s Taken!" % snapshot_iter)

	# Display images
	display_double_image(image_left, image_right)