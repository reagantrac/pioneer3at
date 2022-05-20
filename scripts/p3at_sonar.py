#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud

readings = []

def sonar_reading(data):
	global readings
	rospy.loginfo(data.points)

def read_sonar():
	rospy.init_node("p3at_sonar")
	
	rospy.Subscriber("/RosAria/sonar", PointCloud, sonar_reading)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		rate.sleep()
	
if __name__ == "__main__":
	try:
		read_sonar()
	except rospy.ROSInterruptException:
		pass
