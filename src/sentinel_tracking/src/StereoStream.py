#!/usr/bin/env python
# 05/22/2016 HugoCMU
# Threading class for grabbing image frames fopr PS3 eye 

from threading import Thread
import cv2

class WebcamVideoStream(object):
	def __init__(self,  src=0):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.stream = cv2.VideoCapture(src)
		(self.grabbed, self.frame) = self.stream.read()

		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False

	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return

			# otherwise, read the next frame from the stream
			(self.grabbed, self.frame) = self.stream.read()

	def read(self):
		# return the frame most recently read
		return self.frame
 
	def stop(self):
		# indicate that th ethread should be stopped
		self.stopped = True

# Paths for calibration files
PATH = os.path.expanduser("~") + '/Sentinel/camera_calibration/OpenCVCalibration/'
CALIB_FILENAME_L = PATH + 'calib/camera_calib_data_L.txt'
CALIB_FILENAME_R = PATH + 'calib/camera_calib_data_R.txt'

class StereoStream(object):

	def __init__(self, servo_object):

		# Create WebcamVideoStream objects
		self.cam_thread_left = WebcamVideoStream(src=1).start()
		self.cam_thread_right = WebcamVideoStream(src=0).start()

		# Raw left and right images
		self.raw_image_left = None
		self.raw_image_right = None

		# Rotated left and right images
		self.rotated_image_left = None
		self.rotated_image_right = None

		# Calibrated left and right images
		self.calib_image_left = None
		self.calib_image_right = None

		# Get calibration matrices (3 part tuple)
		# calib_matrix = (mtx, dist, newcameramtx)
		self.calib_matrix_left = load_calibration(CALIB_FILENAME_L)
		self.calib_matrix_right = load_calibration(CALIB_FILENAME_R)

		# Disparity image
		self.disparity = None

		# Servo object associated with tracker
		self.servos = servo_object

	def calibrate_imgs(self):
		'''	
			Uses calibration matrices stored in file to calibrate rotate images
		'''
		# Use OpenCV function to undistort images
		self.calib_image_left = cv2.undistort(self.rotated_image_left, self.calib_matrix_left[0], self.calib_matrix_left[1], None, self.calib_matrix_left[2])
		self.calib_image_right = cv2.undistort(self.rotated_image_right, self.calib_matrix_right[0], self.calib_matrix_right[1], None, self.calib_matrix_right[2])

	def rotate_imgs(self):
		'''
			Rotates left and right images to proper orientation. Note this is done with transpose and flip
			as opposed to cv2.warpAffine to improve speed.
		''' 
		self.image_left = cv2.flip(cv2.transpose(self.raw_image_left), 0)
		self.image_right = cv2.flip(cv2.transpose(self.raw_image_right), 1)

	def get_raw_imgs(self):
		'''
			Grabs the latest camera frame from the Camera threads
		'''
		self.raw_image_left = self.cam_thread_left.read()
		self.raw_image_right = self.cam_thread_right.read()

	def stop_camera_threads(self):
		'''	
			Stops both of the camera threads
		'''
		self.cam_thread_left.stop()
		self.cam_thread_right.stop()

	def __del__():
		self.stop_camera_thread()
