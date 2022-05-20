#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3

def get_gps_point():
	return (115.8171314540043, -31.98082891705035)

def read_gps():

	g = rospy.Publisher('/p3at/gps', Vector3, queue_size=1)
	rospy.init_node("p3at_gps")
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pos = Vector3()
		lon, lat = get_gps_point()
		pos.x = lon
		pos.y = lat
		g.publish(pos)
		rate.sleep()

if __name__ == "__main__":
	try:
		read_gps()
	except rospy.ROSInterruptExeception:
		pass