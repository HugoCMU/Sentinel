#!/usr/bin/env python

import numpy as np
import cv2
from collections import Counter


# camera_r = cv2.VideoCapture(0)
# camera_l = cv2.VideoCapture(1)

# # grab the current frames
# (grabbed, frame_r) = camera_r.read()
# (grabbed, frame_l) = camera_l.read()

# # Convert color images to gray for disparity map
# frame_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
# frame_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)

# # cv2.imshow("Right Image", frame_r)
# # cv2.waitKey(0)
# # cv2.imshow("Left Image", frame_l)
# # cv2.waitKey(0)

# # print frame_r.shape
# # print frame_l.shape

# rot_frame_r = np.zeros((frame_r.shape[1],frame_r.shape[0]), np.uint8)
# rot_frame_l = np.zeros((frame_l.shape[1],frame_l.shape[0]), np.uint8)

# # print rot_frame_r.shape
# # print rot_frame_l.shape

# cv2.transpose(frame_r, rot_frame_r);
# # print rot_frame_r.shape
# rot_frame_r = cv2.flip(rot_frame_r, 1);
# # print rot_frame_r.shape

# cv2.transpose(frame_l, rot_frame_l);
# # print rot_frame_l.shape
# rot_frame_l = cv2.flip(rot_frame_l, 0);
# # print rot_frame_l.shape

# cv2.imwrite('image_right.png',rot_frame_r)
# cv2.imwrite('image_left.png',rot_frame_l)

rot_frame_r = cv2.imread('right_frame_thing.jpg',0)
rot_frame_l = cv2.imread('left_frame_thing.jpg',0)

# cv2.imshow("Right Image", rot_frame_r)
# cv2.waitKey(0)
# cv2.imshow("Left Image", rot_frame_l)
# cv2.waitKey(0)

disparities = [0, 16, 32, 48, 64, 80]
SADWindowSize = [5, 7, 15, 17, 23, 27, 29, 39, 45, 55, 67, 73, 85, 99, 107, 115, 121]

for disp in disparities:

	for sad in SADWindowSize:

		print "(ndisparities, SADWindowSize): (" + str(disp) + " ," + str(sad) + ")"

		stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, ndisparities=disp, SADWindowSize=sad)
		disparity = stereo.compute(rot_frame_l, rot_frame_r, disptype=cv2.CV_32F)

		# norm_coeff = 255 / disparity.max()

		# cv2.namedWindow("Disparity Image", 0)
		# cv2.imshow('Disparity Image', (disparity + 1) * norm_coeff / 255)
		# cv2.waitKey(0)

		cv2.namedWindow("Disparity Image", 0)
		res = cv2.convertScaleAbs(disparity)
		cv2.imshow('Disparity Image', res)
		cv2.waitKey(0)

# Calibrate Camera
# rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.023 right:=/stereo/cv_camera_right_node/image_raw left:=/stereo/cv_camera_left_node/image_raw right_camera:=/stereo/cv_camera_right_node/ left_camera:=/stereo/cv_camera_left_node/
