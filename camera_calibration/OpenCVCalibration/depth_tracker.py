#!/usr/bin/env python

import numpy as np
import cv2
from collections import Counter

import cPickle


# Define wait time for images
IMSHOW_WAIT_TIME = 3000 # 3 seconds

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

def load_calibration(filename):
	'''
		Loads calibration matrices from given file using cPickle
		# TODO: Use JSON files instead
	'''
	with open(CALIB_FILENAME_L, "r") as f:
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

CALIB_FILENAME_L = 'calib/camera_calib_data_L.txt'
CALIB_FILENAME_R = 'calib/camera_calib_data_R.txt'

rot_frame_l = cv2.imread('tmp/imgL_2.png', cv2.CV_LOAD_IMAGE_GRAYSCALE)
rot_frame_r = cv2.imread('tmp/imgR_2.png', cv2.CV_LOAD_IMAGE_GRAYSCALE)

# Load cailbration data
mtx_l, dist_l, newcameramtx_l = load_calibration(CALIB_FILENAME_L)
mtx_r, dist_r, newcameramtx_r = load_calibration(CALIB_FILENAME_R)

img_l_undistorted = cv2.undistort(rot_frame_l, mtx_l, dist_l, None, newcameramtx_l)
img_r_undistorted = cv2.undistort(rot_frame_r, mtx_r, dist_r, None, newcameramtx_r)

# disparities = [0, 16, 32, 48, 64, 80]
# SADWindowSize = [5, 7, 15, 17, 23, 27, 29, 39, 45, 55, 67, 73, 85, 99, 107, 115, 121]

# for disp in disparities:

# 	for sad in SADWindowSize:

# 		print "(ndisparities, SADWindowSize): (" + str(disp) + " ," + str(sad) + ")"

# 		stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, ndisparities=disp, SADWindowSize=sad)
# 		disparity = stereo.compute(rot_frame_l, rot_frame_r, disptype=cv2.CV_32F)

# 		# norm_coeff = 255 / disparity.max()

# 		# cv2.namedWindow("Disparity Image", 0)
# 		# cv2.imshow('Disparity Image', (disparity + 1) * norm_coeff / 255)
# 		# cv2.waitKey(0)

# 		res = cv2.convertScaleAbs(disparity)
# 		display_image(res, frame_name="Disparity Image")

disparities = [16, 32, 48, 64, 80, 96, 112]
SADWindowSize = [10, 15, 20] #, 5, 7, 15, 20, 30] #[5, 7, 15, 17, 23, 27, 29, 39, 45, 55, 67, 73, 85, 99, 107, 115, 121]
uniquenessRatio = [1, 3, 10]
speckleWindowSize = [100, 200]
speckleRange = [20, 5, 1, 30]

for ur in uniquenessRatio:
	for sw in speckleWindowSize:
		for sr in speckleRange:
			for dispa in disparities:
				for sad in SADWindowSize:

					print("--------------------------------------------------------------------")
					print("(numDisparities: " + str(dispa))
					print("(SADWindowSize: " + str(sad))
					print("(uniquenessRatio: " + str(ur))
					print("(speckleWindowSize: " + str(sw))
					print("(speckleRange: " + str(sr))

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

					print("Got past computation")

					disp = stereo.compute(img_r_undistorted, img_l_undistorted).astype(np.float32) / 16.0
					disparity_img = (disp-min_disp)/dispa

					disparity_img = (stereo.compute(img_r_undistorted, img_l_undistorted) / 16.0).astype(np.uint8)

					print("Got past computation")

					print(rot_frame_l.dtype, " ", rot_frame_l.shape)
					print(disparity_img.dtype, " ", disparity_img.shape)

					# Display tuning params in image window
					frame_name = "numDisp=%s, SAD=%s, uniqRat=%s, spekWin=%s, spekRang=%s" % (dispa, sad, ur, sw, sr)
					display_double_image(rot_frame_l, disparity_img, frame_name=frame_name)

# Calibrate Camera
# rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.023 right:=/stereo/cv_camera_right_node/image_raw left:=/stereo/cv_camera_left_node/image_raw right_camera:=/stereo/cv_camera_right_node/ left_camera:=/stereo/cv_camera_left_node/
