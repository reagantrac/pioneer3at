#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3

def keypress(data):
	p = rospy.Publisher("/p3at/keep_alive", Bool, queue_size=1)
	p.publish(True)
	
	v = Vector3()
	if data.data == "w": v.x = 1
	elif data.data == "s": v.x = -1
	
	if data.data == "a": v.y = 1
	elif data.data == "d": v.y = -1
	
	m = rospy.Publisher("/p3at/manual_cmd", Vector3, queue_size=1)
	m.publish(v)
	
	if data.data == " ":
		s = rospy.Publisher("/p3at/switch_mode", Vector3, queue_size=1)
		s.publish(True)

def input():

	rospy.Subscriber('/p3at/key_input', String, keypress)
	rospy.init_node("p3at_input")
	
	rospy.spin()


if __name__ == "__main__":
	try:
		input()
	except rospy.ROSInterruptExeception:
		pass
