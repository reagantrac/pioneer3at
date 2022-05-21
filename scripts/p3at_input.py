#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
from evdev import list_devices, InputDevice, categorize, ecodes

stick_min = 10000
stick_max = 53000

def read_controller(dev):
	event = read_one()
	if event == None: return
	# button
	if event.type == ecodes.EV_KEY:
		print("button", event.code, event.value)

	#read stick axis movement
	elif event.type == ecodes.EV_ABS:
		
		print("stick", event.code, event.value)

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
		s = rospy.Publisher("/p3at/switch_mode", Bool, queue_size=1)
		s.publish(True)
	
	s = rospy.Publisher("/p3at/ui_cmd", String, queue_size=1)
	s.publish(data.data)

def read_input(dev):
	rospy.Subscriber('/p3at/key_input', String, keypress)
	rospy.init_node("p3at_input")
	
	
	rospy.spin()


if __name__ == "__main__":
	dev = InputDevice( list_devices()[0] )
	try:
		read_input(dev)
	except rospy.ROSInterruptExeception:
		pass
