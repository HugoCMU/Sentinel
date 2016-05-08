#!/usr/bin/env python
# 03/14/2016 HugoCMU
# Program will track humans in the frame

# Allow compatibility with Python3
from __future__ import print_function

# ROS related imports
import roslib
roslib.load_manifest('sentinel_tracking')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Import some standard stuff
import numpy as np
import sys

# Import OpenCV and imutils (package for HOG people detection)
import cv2
from cv_bridge import CvBridge, CvBridgeError
# from imutils.object_detection import non_max_suppression
# from imutils import paths
# import imutils

# Import framework to deal with servos
from servo_talker import servos

# Define camera rotation parameters
LEFT_ROTATION_ANGLE = 90
RIGHT_ROTATION_ANGLE = -90

# Define wait time for images
IMSHOW_WAIT_TIME = 1000 # 3 seconds

def cv_size(img):
	'''
		Function returns image size (not a tuple)
	'''
	return img.shape[1::-1]

class tracker(object):

	def __init__(self, servo_object):

		# Create camera capture objects
		self.cam_left = cv2.VideoCapture(0)
		self.cam_right = cv2.VideoCapture(1)

		# Raw left and right images
		self.raw_image_left = None
		self.raw_image_right = None

		# Rotated left and right images
		self.image_left = None
		self.image_right = None

		# Servo object associated with tracker
		self.servos = servo_object

	def __str__(self):
		'''
			Function will show the stereo disparity map
		'''

		# Print out raw image sizes
		print("Raw Left Image Size: " + str(tuple(cv_size(self.raw_image_left))))
		print("Raw Right Image Size: " + str(tuple(cv_size(self.raw_image_right))))

		# Print out rotated image sizes
		print("Rotated Left Image Size: " + str(tuple(cv_size(self.image_left))))
		print("Rotated Right Image Size: " + str(tuple(cv_size(self.image_right))))

		# # Display Raw Left image for testing
		# cv2.imshow("Raw Left Image", self.raw_image_left)
		# cv2.waitKey(IMSHOW_WAIT_TIME)

		# # Display image for testing
		# cv2.imshow("Raw Right Image", self.raw_image_right)
		# cv2.waitKey(IMSHOW_WAIT_TIME)

		# # Display Raw Left image for testing
		# cv2.imshow("Rotated Left Image", self.image_left)
		# cv2.waitKey(IMSHOW_WAIT_TIME)

		# # Display image for testing
		# cv2.imshow("Rotated Right Image", self.image_right)
		# cv2.waitKey(IMSHOW_WAIT_TIME)

	def depth_map(self):
		'''
			Function will show the stereo disparity map
		'''

		print("Creating depth_map")

		# Convert color images to gray for disparity map
		image_right_mono = cv2.cvtColor(self.image_right, cv2.COLOR_BGR2GRAY)
		image_left_mono = cv2.cvtColor(self.image_left, cv2.COLOR_BGR2GRAY)

		# Display image for testing
		cv2.imshow("Monocromatic Right Image", image_right_mono)
		cv2.waitKey(IMSHOW_WAIT_TIME)
		# Display image for testing
		cv2.imshow("Monocromatic Image", image_left_mono)
		cv2.waitKey(IMSHOW_WAIT_TIME)

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

							# Get left and right images
							self.grab_left()
							self.grab_right()

							# Rotate raw images
							self.rotate_imgs()

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


							disp = stereo.compute(self.image_left, self.image_right).astype(np.float32) / 16.0

							print("Got past computation")

							cv2.imshow('disparity', (disp-min_disp)/dispa)
							cv2.waitKey(500)


		# # Compute disparity image using StereoBM function
		# stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, ndisparities=0, SADWindowSize=23)
		# disparity = stereo.compute(image_left_mono, image_right_mono, disptype=cv2.CV_32F)
		# disparity_clean = cv2.convertScaleAbs(disparity)

		# # Display image for testing
		# cv2.namedWindow("Disparity Image", 0)
		# cv2.imshow('Disparity Image', disparity_clean)
		# cv2.waitKey(3)

		# return disparity_clean

	def depth_tracking(self):
		'''
			Tracking function based on depth sensing. Function will publish servo
			messages to track a human in the frame.
		'''

		print("Depth Tracking Function enabled")

		# Get left and right images
		self.grab_left()
		self.grab_right()

		# Rotate raw images
		self.rotate_imgs()

		# Print out class attributes using __str__ function
		self.__str__()

		# Create disparity map from images
		self.depth_map()

	def rotate_imgs(self):
		'''
			Rotates left and right images to proper orientation. Note this is done with transpose and flip
			as opposed to cv2.warpAffine to improve speed.
		''' 

		# Rotate left image
		self.image_left = cv2.transpose(self.raw_image_left)
		self.image_left = cv2.flip(self.image_left, 0)

		# Rotate right image
		self.image_right = cv2.transpose(self.raw_image_right)
		self.image_right = cv2.flip(self.image_right, 1)

	def grab_left(self):
		'''
			Grabs frame from left camera
		''' 

		# Grab image directly from cv2 video capture object
		foo, cv_image = self.cam_left.read()

		# Store image in object
		self.raw_image_left = cv_image

	def grab_right(self):
		'''
			Grabs frame from right camera
		'''

		# Grab image directly from cv2 video capture object
		foo, cv_image = self.cam_right.read()

		# Store image in object
		self.raw_image_right = cv_image

def main(args):

	# Initialize node
	rospy.init_node('tracking', anonymous=True)

	# Initialize servo object
	servo_object = servos()
	servo_object.home_servos()

	# Initialize tracking object
	tracking_object = tracker(servo_object)

	# Print out depth image
	tracking_object.depth_tracking()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)