#!/usr/bin/env python
# 03/14/2016 HugoCMU
# Program will track nearby object in the camera frame

from __future__ import print_function

import roslib
roslib.load_manifest('sentinel_tracking')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from matplotlib import pyplot as plt

from servo_talker import servos

class tracker:

	def __init__(self):

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/stereo/cv_camera_right_node/image_raw", Image,self.callback_left)
		self.image_sub = rospy.Subscriber("/stereo/cv_camera_left_node/image_raw", Image,self.callback_right)
		self.image_left = None
		self.image_right = None

	def callback_left(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.image_left = cv_image

		# # Print out message and display image for testing
		# print("Left image stored")
		# cv2.imshow("Left Image", self.image_left)
		# cv2.waitKey(3)

	def callback_right(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.image_right = cv_image

		# # Print out message and display image for testing
		# print("Right image stored")
		# cv2.imshow("Right Image", self.image_right)
		# cv2.waitKey(3)

	def depth_map(self):
		'''
			Function will show the stereo disparity map
		'''

		print("Creating depth_map")

		# Convert color images to gray for disparity map
		image_right_mono = cv2.cvtColor(self.image_right, cv2.COLOR_BGR2GRAY)#CV_BGR2GRAY)
		image_left_mono = cv2.cvtColor(self.image_left, cv2.COLOR_BGR2GRAY)#CV_BGR2GRAY)

		stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, ndisparities=32, SADWindowSize=15)
		disparity = stereo.compute(image_left_mono, image_right_mono)

		# cv2.imshow("Left Raw Image", self.image_left)
		# cv2.imshow("Right Raw Image", self.image_right)
		# cv2.imshow("Left Image", image_left_mono)
		# cv2.imshow("Right Image", image_right_mono)
		# cv2.imshow("Right Image", disparity)
		# cv2.waitKey(3)

		# plt.imshow(self.image_left,'gray')
		# plt.imshow(self.image_right,'gray')
		plt.imshow(disparity,'gray')
		plt.show()

def main(args):

	# Initialize tracking object
	tracking_object = tracker()

	# Initialize node
	rospy.init_node('tracking', anonymous=True)

	# Initialize servo object
	servo_object = servos()
	servo_object.home_servos()

	try:
		tracking_object.depth_map()
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)