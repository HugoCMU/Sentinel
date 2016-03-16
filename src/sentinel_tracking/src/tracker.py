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
from imutils.object_detection import non_max_suppression
from imutils import paths
import imutils

# Import framework to deal with servos
from servo_talker import servos

class tracker:

	def __init__(self, servo_object):

		self.bridge = CvBridge()
		# Subscribe to mono (grey) image directly to increase speed (no need to convert)
		self.image_sub_left = rospy.Subscriber("/stereo/right/image_mono", Image,self.callback_left)
		self.image_sub_right = rospy.Subscriber("/stereo/left/image_mono", Image,self.callback_right)
		self.image_left = None
		self.image_right = None

		# State variables track whether messages are up-to-date
		self.state_left = False
		self.state_right = False

		# Servo object associated with tracker
		self.servos = servo_object

	def depth_map(self):
		'''
			Function will show the stereo disparity map
		'''

		print("Creating depth_map")

		# Convert color images to gray for disparity map
		# image_right_mono = cv2.cvtColor(self.image_right, cv2.COLOR_BGR2GRAY)#CV_BGR2GRAY)
		# image_left_mono = cv2.cvtColor(self.image_left, cv2.COLOR_BGR2GRAY)#CV_BGR2GRAY)

		# Compute disparity image using StereoBM function
		stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, ndisparities=0, SADWindowSize=23)
		disparity = stereo.compute(self.image_left, self.image_right, disptype=cv2.CV_32F)
		disparity_clean = cv2.convertScaleAbs(disparity)

		# # Display image for testing
		# cv2.namedWindow("Disparity Image", 0)
		# cv2.imshow('Disparity Image', disparity_clean)
		# cv2.waitKey(0)

		return disparity_clean

	def people_detect(self, image):
		'''
			Function detects people in image using HOG function form imutils.
			Returns images with rectangle around people, and people locations (pixels) on the image
		'''

		print("Detecting people function enabled")

		# Initialize the HOG descriptor/person detector
		hog = cv2.HOGDescriptor()
		hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

		# Resize image to reduce detection time and improve detection accuracy
		image_resize = imutils.resize(image, width=min(400, image.shape[1]))
		orig = image.copy()

		# Detect people in the image
		(rects, weights) = hog.detectMultiScale(image_resize, winStride=(4, 4), padding=(8, 8), scale=1.05)

		# Apply non-maxima suppression to the bounding boxes to get rid of sub-boxes
		rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
		nms_rects = non_max_suppression(rects, probs=None, overlapThresh=0.65)

		# Draw the bounding boxes on the original image
		for (xA, yA, xB, yB) in nms_rects:
			cv2.rectangle(orig, (xA, yA), (xB, yB), (0, 255, 0), 2)

		# # Display image for testing
		# cv2.imshow("HOG People Detector", orig)
		# cv2.waitKey(0)

		# Determine center location for each rectangle
		centers = [[rect[0] + (rect[2] - rect[0])/2, rect[1] + (rect[3] - rect[1])/2] for rect in nms_rects]

		return orig, centers

	def main_tracking(self):
		'''
			The main tracking function. Invoked once state tracking variables are set in the callback functions
			(There has to be a better/"ROS" way to do this, but haven't found out yet). Function will publish servo
			messages to track a human in the frame.
		'''

		# Re-assign state variables
		self.state_left = False
		self.state_right = False

		print("Main Tracking Function enabled")

		# Get people in both camera frames
		track_image_left, centers_left = self.people_detect(self.image_left)
		track_image_right, centers_right = self.people_detect(self.image_right)

		# Combine center locations (or people) for both cameras
		centers = centers_left + centers_right

		print(centers_left)
		print(centers_right)
		print(centers)

		# Make sure that there is some kind of human being tracked
		if not centers:
			print("No humans found")
			return # If no human, do not move camera

		# Set the middle pixle location for an image (based on image size)
		middle_pixle = [480/2, 640/2]

		# TODO: Publish the images showing tracking rectangles

		# Choose a random person
		person = centers[0]

		# Get difference between center of person rectangle and center of the images
		diff = [person[0] - middle_pixle[0], person[1] - middle_pixle[1]] 

		print("diff: " + str(diff[0]) + " , " + str(diff[1]))

		# # Sleep for some time to prevent flooding servos with messages
		# rate = rospy.Rate(1)
		# rate.sleep()

		# Use servo object to move servos by difference
		self.servos.move_servos(diff)

	def callback_left(self,data):
		'''
			Callback function for left camera
		'''

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)

		print("callback_left")

		# Store image in object
		self.image_left = cv_image

		# # Display image for testing
		# cv2.imshow("Left Image", self.image_left)
		# cv2.waitKey(3)

		# Update state variable, advertising "fresh" image
		self.state_left = True

		# If both images are up-to-date
		if all([self.state_left, self.state_right]):

			# Run our main tracking function
			self.main_tracking()

	def callback_right(self,data):
		'''
			Callback function for right camera
		'''

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)

		print("callback_right")

		# Store image in object
		self.image_right = cv_image

		# # Display image for testing
		# cv2.imshow("Right Image", self.image_right)
		# cv2.waitKey(3)

		# Update state variable, advertising "fresh" image
		self.state_right = True

		# If both images are up-to-date
		if all([self.state_left, self.state_right]):

			# Run our main tracking function
			self.main_tracking()

def main(args):

	# Initialize node
	rospy.init_node('tracking', anonymous=True)

	# Initialize servo object
	servo_object = servos()
	servo_object.home_servos()

	# Initialize tracking object
	tracking_object = tracker(servo_object)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)