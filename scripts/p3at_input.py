#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
from evdev import list_devices, InputDevice, categorize, ecodes

stick_min = 10000
stick_max = 53000

drive_vec = Vector3()
key_pressed = ""
alive = False

def read_controller(dev):
	global key_pressed, drive_vec

	
	while dev.read_one() != None: 
		event = dev.read_one()
		if event == None:
			return
		# button
		if event.type == ecodes.EV_KEY:
			#if event.code == 310: drive_vec = Vector3()
			
			alive = rospy.Publisher("/p3at/keep_alive", Bool, queue_size=1)
			alive.publish(True)
			print("button", event.code, event.value)

		#read stick axis movement
		elif event.type == ecodes.EV_ABS:
			if event.code == 0:
				y = event.value - (stick_max)/2 - stick_min
				y /= stick_max
				drive_vec.y = round(-y,1)/2
				key_pressed = "k"
			if event.code == 1:
				x = event.value - (stick_max)/2 - stick_min
				x /= stick_max
				drive_vec.x = round(-x,1)/2
		key_pressed = str(event.code)
		

def keypress(data):
	global key_pressed, drive_vec
	
	if data.data == "w": drive_vec.x = 1
	elif data.data == "s": drive_vec.x = -1
	else: drive_vec.x = 0
	
	if data.data == "a": drive_vec.y = 1
	elif data.data == "d": drive_vec.y = -1
	else: drive_vec.y = 0
	
	key_pressed = data.data

def read_input(dev):
	global key_pressed, drive_vec

	ui = rospy.Publisher("/p3at/ui_cmd", String, queue_size=1)
	cmd = rospy.Publisher("/p3at/manual_cmd", Vector3, queue_size=1)
	mode = rospy.Publisher("/p3at/switch_mode", Bool, queue_size=1)
	alive = rospy.Publisher("/p3at/keep_alive", Bool, queue_size=1)

	#rospy.Subscriber('/p3at/key_input', String, keypress)
	rospy.init_node("p3at_input")
	
	
	
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		read_controller(dev)
		if key_pressed == "": 
			cmd.publish(drive_vec)
			continue
			
		
		#alive.publish(True)
		ui.publish(key_pressed)
		cmd.publish(drive_vec)
		if key_pressed == " " or key_pressed == 310:
			mode.publish(True)
			
		key_pressed = ""
		rate.sleep()


if __name__ == "__main__":
	try:
		dev = InputDevice( list_devices()[0] )
		read_input(dev)
	except rospy.ROSInterruptExeception:
		pass
