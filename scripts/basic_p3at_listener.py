#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Data recieved: " + data.data)

def listener():
	rospy.init_node("p3at_listener")
	rospy.Subscriber("/RosAria/pose", Odometry, callback)

	#prevent termination while still running
	rospy.spin()


if __name__ == "__main__":
	listener()
