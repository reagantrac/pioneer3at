#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

def drive_forward():
	pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
	rospy.init_node("drive_test")
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		twist = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
		pub.publish(twist)
		rate.sleep()


if __name__ == "__main__":
	try:
		drive_forward()
	except rospy.ROSInterruptExeception:
		pass
