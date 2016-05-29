#!/usr/bin/env python
# 05/24/2016 HugoCMU
# Threading class for grabbing image frames fopr PS3 eye 

import numpy as np
import rospy
import cv2

# Import custom classes to deal with servos and camera threads
from Servos import Servos
from StereoStream import StereoStream

# Size of squares for our tracking function
SQUARE_SIZE = 0.2 # As a percentage of the width/height

IMSHOW_WAIT_TIME = 3000 # 3 seconds wait time when showing images
COLOR = (255, 255, 255) # Color of overlayed text/lines on images
VEC_SIZE = 100 # Size of overlayed movement vector on images (~pixels)

# Servo object associated with tracker
servos = Servos()
servos.home_servos() # Start with servos in home position

# Create camera stream object to get disparity images
stereo_thread = StereoStream()

# Becomes true once first disparity image dimmensions have been recorded
dim_check = False
img_height, img_width, row_edges, col_edges = 0, 0, [], []

# The underlying algorithm draws a 3x3 grid around the center of the camera,
# where the size of each square is based on SQUARE_SIZE and image dimmensions.
# Numbering of the grid is as follows:
#	1 2 3
#	4 5 6
#	7 8 9
# Servo Conventions for movement are as follows [servo1, servo2]:
# [ 1, 0] - turns head right (POV of head)
# [-1, 0] - turns head left (POV of head)
# [ 0, 1] - turns head up
# [ 0,-1] - turns head down

# Direction of servo movement for each grid square
MOVE_VECT = [
		(-1, 1), ( 0, 1), ( 1, 1),
		(-1, 0), ( 0, 0), ( 1, 0),
		(-1,-1), ( 0,-1), ( 1,-1)
]

def display_image(image, frame_name="Image"):
	'''
		Displays given image in a named OpenCV window
	'''
	cv2.imshow(frame_name, image)
	cv2.moveWindow(frame_name, 30, 30) # Coordinate (X,Y) = (30,30)
	cv2.waitKey(IMSHOW_WAIT_TIME)
	cv2.destroyAllWindows()

def display_tracker(image, scores, vector):
	'''
		Overlays the 3x3 grid, disparity scores, and movement vector over an image
	'''
	# Draw out grid
	for column in col_edges:
		cv2.line(image, (column, row_edges[0]), (column, row_edges[3]), COLOR, 1)
	for row in row_edges:
		cv2.line(image, (col_edges[0], row), (col_edges[3], row), COLOR, 1)

	# Draw disparity score for each grid square
	for s, c, r in zip(range(9), range(3)*3, [1, 1, 1, 2, 2, 2, 3, 3, 3]):
		cv2.putText(image, str(round(scores[s],0)), (col_edges[c], row_edges[r]), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR, 2)

	# Draw movement direction vector
	cv2.line(image, (int(img_width/2.0), int(img_height/2.0)),
				 (int(img_width/2.0 + VEC_SIZE*vector[0]), int(img_height/2.0 - VEC_SIZE*vector[1])), COLOR, 1)

	display_image(image, "Disparity Tracker Overlay")

def show_disparity():
	'''
		Shows disparity image (but keeps the camera still)
	'''
	# Get most recent disparity image
	disparity = stereo_thread.get_disparity()
	display_image(disparity, frame_name="Disparity Simple")

def track_close_object(display=False):
	'''
		Function will track the closest object to the cameras using the disparity image,
		returns vector of tracking direction < x,y > where -1 < x,y < 1
	'''

	global dim_check, img_height, img_width, row_edges, col_edges

	# Get most recent disparity image
	disparity = stereo_thread.get_disparity()

	# If dimmensions of image have not yet been seen (first pass)
	if not dim_check:

		img_height, img_width = disparity.shape

		# Define the grid box edge locations (3x3 grid has 8 "edges")
		row_edges = [
			int(img_height/2.0-(1.5*SQUARE_SIZE*img_height)),
			int(img_height/2.0-(0.5*SQUARE_SIZE*img_height)),
			int(img_height/2.0+(0.5*SQUARE_SIZE*img_height)),
			int(img_height/2.0+(1.5*SQUARE_SIZE*img_height))
		]

		col_edges = [
			int(img_width/2.0-(1.5*SQUARE_SIZE*img_width)),
			int(img_width/2.0-(0.5*SQUARE_SIZE*img_width)),
			int(img_width/2.0+(0.5*SQUARE_SIZE*img_width)),
			int(img_width/2.0+(1.5*SQUARE_SIZE*img_width))
		]
		
		dim_check = True

	rospy.logdebug("sentinel_tracking.py: img_height, img_width: %s, %s" % (img_height, img_width))
	rospy.logdebug("sentinel_tracking.py: row_edges: %s" % row_edges)
	rospy.logdebug("sentinel_tracking.py: col_edges: %s" % col_edges)

	# Sub-select each grid square from the disparity image
	grid = [
		disparity[row_edges[0]:row_edges[1], col_edges[0]:col_edges[1]],
		disparity[row_edges[0]:row_edges[1], col_edges[1]:col_edges[2]],
		disparity[row_edges[0]:row_edges[1], col_edges[2]:col_edges[3]],
		disparity[row_edges[1]:row_edges[2], col_edges[0]:col_edges[1]],
		disparity[row_edges[1]:row_edges[2], col_edges[1]:col_edges[2]],
		disparity[row_edges[1]:row_edges[2], col_edges[2]:col_edges[3]],
		disparity[row_edges[2]:row_edges[3], col_edges[0]:col_edges[1]],
		disparity[row_edges[2]:row_edges[3], col_edges[1]:col_edges[2]],
		disparity[row_edges[2]:row_edges[3], col_edges[2]:col_edges[3]]
	]

	# Average disparity score for each square
	disparity_grid_scores = [np.mean(square) for square in grid]

	# Combine average scores with movement vectors to get a single movement vector
	weighted_vectors = [tuple(i * score for i in vector) for vector, score in zip(MOVE_VECT, disparity_grid_scores)]
	movement_vector = np.mean(weighted_vectors, axis=0)

	if display:
		display_tracker(disparity, disparity_grid_scores, movement_vector)

	rospy.logdebug("sentinel_tracking.py: Weighted_vectors, %s" % weighted_vectors)
	rospy.loginfo("Movement vector, %s" % movement_vector)

	return movement_vector

for _ in range(500):

	# show_disparity()

	track_vector = track_close_object(display=True)
	servos.move_servos(track_vector)
