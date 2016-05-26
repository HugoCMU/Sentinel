#!/usr/bin/env python
# 05/24/2016 HugoCMU
# Threading class for grabbing image frames fopr PS3 eye 

from __future__ import print_function
import numpy as np

# Import custom classes to deal with servos and camera threads
from Servos import Servos
from StereoStream import StereoStream

# Size of squares for our tracking function
SQUARE_SIZE = 0.2 # As a percentage of the width/height

# Servo object associated with tracker
servos = Servos()

# Create camera stream object to get disparity images
stereo_thread = StereoStream()

def track_close_object():
	'''
		Function will track the closest object to the cameras using the disparity image
	'''

	# Get most recent disparity image
	disparity = stereo_thread.get_disparity()

	# min and max values of disparity image
	min_value = np.amin(disparity) 
	max_value = np.amax(disparity)
	delta = max_value - min_value

	col_avg = np.mean(disparity, axis=0) # Column average
	row_avg = np.mean(disparity, axis=1) # Row average

	img_height, img_width = disparity.shape

	# Draw a 3x3 grid around the center of the camera, size of square is based on SQUARE_SIZE and image dimmensions
	#	123
	#	456
	#	789

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

	# Movement vectors for each square in the grid
	movement_vect = [
			(-1, -1), (-1, 0), (-1, 1)
			( 0, -1), ( 0, 0), ( 0, 1)
			( 1, -1), ( 1, 0), ( 1, 1)
	]

	# Sub-select the relevant grid squares of the disparity image
	grid = [
		disparity[row_edges[0]:row_edges[1], column_edges[0]:column_edges[1]],
		disparity[row_edges[0]:row_edges[1], column_edges[1]:column_edges[2]],
		disparity[row_edges[0]:row_edges[1], column_edges[2]:column_edges[3]],
		disparity[row_edges[1]:row_edges[2], column_edges[0]:column_edges[1]],
		disparity[row_edges[1]:row_edges[2], column_edges[1]:column_edges[2]],
		disparity[row_edges[1]:row_edges[2], column_edges[2]:column_edges[3]],
		disparity[row_edges[2]:row_edges[3], column_edges[0]:column_edges[1]],
		disparity[row_edges[2]:row_edges[3], column_edges[1]:column_edges[2]],
		disparity[row_edges[2]:row_edges[3], column_edges[2]:column_edges[3]]
	]


	# Average disparity score for each square
	disparity_grid_scores = [np.mean(square) for square in grid]

	# Combine average scores with movement vectors to get a single movement vector
	weighted_vectors = [tuple(i * score for i in vector) for vector, score in zip(movement_vect, disparity_grid_scores)]
	vector = np.mean(weighted_vectors, axis=0)

	# Convert movement vector into format to feed servo tracker


	# Move servos accordingly


track_close_object()