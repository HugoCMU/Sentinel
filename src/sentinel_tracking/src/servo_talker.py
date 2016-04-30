#!/usr/bin/env python
# 03/10/2016 HugoCMU
# Program will publish given servo angles as servo messages

import rospy
from std_msgs.msg import UInt16

class servos(object):

	def __init__(self):

		# Initialize publishers to servo commands
		self.pub_1 = rospy.Publisher('/servo1', UInt16, queue_size=10)
		self.pub_2 = rospy.Publisher('/servo2', UInt16, queue_size=10)

		# Set angles to default angles (set in launch file)
		self.angles = [rospy.get_param("servo1_default"), rospy.get_param("servo2_default")]

	def test_servos(self):
		'''
			Function will cycle servos through a bunch of angles for testing
		'''

		rospy.init_node('talker', anonymous=True)
		rate = rospy.Rate(10) # 10hz

		# Initialize angles to cycle through in test
		angles_1 = range(180)
		angles_2 = range(90) * 2

		for i in xrange(len(angles_1)):

			rospy.loginfo(angles_1[i])
			rospy.loginfo(angles_2[i])

			self.pub_1.publish(angles_1[i])
			self.pub_2.publish(angles_2[i])

			rate.sleep()

	def move_servos(self, angle_diff):
		'''
			Function moves servos by angle increment
		'''

		# Multiply raw difference reading (pixels) by the gain
		diff_servo1 = rospy.get_param("P_gain") * -1 * angle_diff[0]
		diff_servo2 = rospy.get_param("P_gain") * -1 * angle_diff[1]

		# Use set_servo function to set new angles
		self.set_servos([self.angles[0] + diff_servo1, self.angles[1] + diff_servo2])

	def home_servos(self):
		'''
			Function returns the servos to their home positions
		'''

		# Use set_servos method and global default angles
		self.set_servos([rospy.get_param("servo1_default"), rospy.get_param("servo2_default")])

	def check_angles(self, angles):
		'''
			Function returns true if the angles are outside allowable servo range
		'''

		# Check to see if angles are within range, printing message if not
		if angles[0] < rospy.get_param("servo1_min"):
			print "Servo 1 ( " + str(angles[0]) + " ) not in range (" + str(rospy.get_param("servo1_min")) + ", " + str(rospy.get_param("servo1_max")) + ")"
			return True

		if angles[0] > rospy.get_param("servo1_max"):
			print "Servo 1 ( " + str(angles[0]) + " ) not in range (" + str(rospy.get_param("servo1_min")) + ", " + str(rospy.get_param("servo1_max")) + ")"
			return True

		if angles[1] < rospy.get_param("servo2_min"):
			print "Servo 2 ( " + str(angles[1]) + " ) not in range (" + str(rospy.get_param("servo2_min")) + ", " + str(rospy.get_param("servo2_max")) + ")"
			return True

		if angles[1] > rospy.get_param("servo2_max"):
			print "Servo 2 ( " + str(angles[1]) + " ) not in range (" + str(rospy.get_param("servo2_min")) + ", " + str(rospy.get_param("servo2_max")) + ")"
			return True

		# All the angles are within range
		return False

	def set_servos(self, angles):
		'''
			Function publishes servo messages with given angles, messages
			will be heard by rosserial_arduino node
		'''

		# Make sure angles are within allowable range
		if self.check_angles(angles):
			return

		print "Setting servos from (" + str(self.angles[0]) + ", " + str(self.angles[1]) + ") to (" + str(angles[0]) + ", " + str(angles[1]) + ")"

		# Update global variable
		self.angles = angles

		# Log and Publish angle message
		rospy.loginfo(angles[0])
		rospy.loginfo(angles[1])
		self.pub_1.publish(angles[0])
		self.pub_2.publish(angles[1])


if __name__ == '__main__':
	try:
		servo_obj = servos() 
		test_servos(servo_obj)
	except rospy.ROSInterruptException:
		pass