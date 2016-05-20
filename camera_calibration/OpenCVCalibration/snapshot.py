#!/usr/bin/env python
# 03/14/2016 HugoCMU
# Program will take a snapshot with both cameras

from __future__ import print_function

import cv2
import os
import time

import sys


# ----------------- GET PROPER FILENAMES & PATHS

# Make sure we use proper directory
os.chdir('$HOME/Sentinel/camera_calibration/OpenCVCalibration/')

# Define name of camera calibration images: calib_image_right_XXX.png
filename = 'img'

# ----------------- TAKE USER INPUT

try:
	pts, args = getopt.getopt(argv,"op")
except getopt.GetoptError:
	print("\n")
	print("-o 	overwrite current generated images in tmp")
	print("-p  push generated images into new date folder")
	print("\n")

	sys.exit(2)

img_iter = 0

for opt, arg in opts:
	if opt in ("-o"):
		pass
	elif opt in ("-p"): # TODO: Allow for long option that defines folder name

		filename = "hist/" + time.strftime("%d/%m/%Y_%H:%M:%S") + "/" + filename

if not opts:
	while os.path.exists(filename_right + '%s.png' % img_iter):
		img_iter += 1

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

# Create camera capture objects
cam_left = cv2.VideoCapture(0)
cam_right = cv2.VideoCapture(1)

# Keep taking pictures until user exits the program
while True:

	# Give user time to move board
	for i in xrange(3):
		print("%s.....", (3 - i))
		time.sleep(1)

	# Grab image directly from cv2 video capture object
	foo, cv_image_left = cam_left.read()
	foo, cv_image_right = cam_right.read()

	# Convert color images to gray for calibration
	image_right_mono = cv2.cvtColor(cv_image_right, cv2.COLOR_BGR2GRAY)
	image_left_mono = cv2.cvtColor(cv_image_left, cv2.COLOR_BGR2GRAY)

	# Rotate images
	image_right, image_left = rotate_imgs(image_right_mono, image_left_mono)

	# Save images to file
	cv2.imwrite(filename + 'R_%s.png' % snapshot_iter, image_right)
	cv2.imwrite(filename + 'L_%s.png' % snapshot_iter, image_left)

	# Increase image counter
	snapshot_iter += 1

	# Visual confirmation
	print("Snapshot - %s", snapshot_iter)




