#!/usr/bin/env python
# 03/10/2016 HugoCMU
# Program will publish given servo angles


import rospy
from std_msgs.msg import UInt16

def test_talker():
	'''
		Function will cycle servos through a bunch of angles for testing
	'''

	# Initialize messages to publish
	pub_1 = rospy.Publisher('/servo1', UInt16, queue_size=10)
	pub_2 = rospy.Publisher('/servo2', UInt16, queue_size=10)

	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	# Initialize angles to cycle through in test
	angles_1 = range(180)
	angles_2 = range(90) * 2

	for i in range(len(angles_1)):

		rospy.loginfo(angles_1[i])
		rospy.loginfo(angles_2[i])

		pub_1.publish(angles_1[i])
		pub_2.publish(angles_2[i])

		rate.sleep()

# def talker(angles):

# 	# If this is in test mode, 
# 	servo1, servo2 = angles

# 	# Initialize messages to publish
# 	pub_1 = rospy.Publisher('/servo1', UInt16, queue_size=10)
# 	pub_2 = rospy.Publisher('/servo2', UInt16, queue_size=10)

# 	rospy.init_node('talker', anonymous=True)
# 	rate = rospy.Rate(10) # 10hz

# 	angles_1 = range(180)
# 	angles_2 = range(60)

# 	counter

# 	while not rospy.is_shutdown():

# 		angle_1 = 60
# 		angle_2 = 60

# 		rospy.loginfo(angle_1)
# 		rospy.loginfo(angle_2)

# 		pub_1.publish(angle_1)
# 		pub_2.publish(angle_2)

# 		rate.sleep()

if __name__ == '__main__':
	try:
		test_talker()
	except rospy.ROSInterruptException:
		pass
