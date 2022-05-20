#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

linear = 0
angular = 0
timer = 0

def receive_command(data):
	global linear, angular, timer
	linear = data.x
	angular = data.y
	timer = 0

def drive():
	global linear, angular, timer

	cmd = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
	rospy.init_node("p3at_drive")
	
	rospy.Subscriber("/p3at/drive_cmd", Vector3, receive_command)
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		timer += 0.1
		if timer > 0.3:
			linear = 0
			angular = 0
		vel = Twist(Vector3(linear, 0, 0), Vector3(0, 0, angular))
		cmd.publish(vel)
		rate.sleep()


if __name__ == "__main__":
	try:
		drive()
	except rospy.ROSInterruptExeception:
		pass
