#!/usr/bin/env python
# 03/10/2016 HugoCMU
# Program will publish given servo angles as servo messages

import rospy
from std_msgs.msg import UInt16

class servos:

	def __init__(self):

		self.pub_1 = rospy.Publisher('/servo1', UInt16, queue_size=10)
		self.pub_2 = rospy.Publisher('/servo2', UInt16, queue_size=10)
		# Define global servo angle variables
		self.angles_default = [45, 45]

		# Define global default servo angles
		self.angles = self.angles_default



	def test_servos(self):
		'''
			Function will cycle servos through a bunch of angles for testing
		'''

		rospy.init_node('talker', anonymous=True)
		rate = rospy.Rate(10) # 10hz

		# Initialize angles to cycle through in test
		angles_1 = range(180)
		angles_2 = range(90) * 2

		for i in range(len(angles_1)):

			rospy.loginfo(angles_1[i])
			rospy.loginfo(angles_2[i])

			self.pub_1.publish(angles_1[i])
			self.pub_2.publish(angles_2[i])

			rate.sleep()

	def move_servos(self, angle_diff):
		'''
			Function moves servos by angle increment
		'''

		# Use set_servo function to set new angles
		set_servos([self.angles[0] + angle_diff[0], self.angles[1] + angle_diff[1]])

	def home_servos(self):
		'''
			Function returns the servos to their home positions
		'''

		# Use set_servos method and global default angles
		self.set_servos(self.angles_default)


	def set_servos(self, angles):
		'''
			Function publishes servo messages with given angles, messages
			will be heard by rosserial_arduino node
		'''

		print "Setting servos to (" + str(angles[0]) + ", " + str(angles[1]) + ")"

		# Update global variable
		self.angles = angles

		# Publish angle message
		rospy.loginfo(angles[0])
		rospy.loginfo(angles[1])
		self.pub_1.publish(angles[0])
		self.pub_2.publish(angles[1])


if __name__ == '__main__':
	try:
		test_servos()
	except rospy.ROSInterruptException:
		pass