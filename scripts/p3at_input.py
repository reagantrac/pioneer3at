#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
from evdev import list_devices, InputDevice, categorize, ecodes

stick_min = 10000
stick_max = 53000

drive_vec = Vector3()
command = ""
alive = False

def read_controller(dev):
	global command, drive_vec

	
	while dev.read_one() != None: 
		event = dev.read_one()
		if event == None:
			return
		# button
		if event.type == ecodes.EV_KEY:
			#if event.code == 310: drive_vec = Vector3()
			
			alive = rospy.Publisher("/p3at/keep_alive", Bool, queue_size=1)
			alive.publish(True)
			pass

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
		#key_pressed = str(event.code)
		

def keypress(data):
	global command, drive_vec
	
	if data.data == "w": drive_vec.x = 1
	elif data.data == "s": drive_vec.x = -1
	else: drive_vec.x = 0
	
	if data.data == "a": drive_vec.y = 1
	elif data.data == "d": drive_vec.y = -1
	else: drive_vec.y = 0
	
	# flag: keyboard and controller events are different
	drive_vec.z = 1 
	
	if data.data == "q":
		command = "ui_waypoint_switch"
	elif data.data == "e":
		command = "ui_waypoint_add_remove"
	elif data.data == "c":
		command = "ui_waypoint_loop_toggle"
	elif data.data == "x":
		command = "ui_waypoint_execute"
	elif data.data == "z":
		command = "ui_waypoint_cancel"

def read_input(dev):
	global command, drive_vec

	ui = rospy.Publisher("/p3at/ui_cmd", String, queue_size=1)
	cmd = rospy.Publisher("/p3at/manual_cmd", Vector3, queue_size=1)
	mode = rospy.Publisher("/p3at/switch_mode", Bool, queue_size=1)
	alive = rospy.Publisher("/p3at/keep_alive", Bool, queue_size=1)

	rospy.Subscriber('/p3at/key_input', String, keypress)
	rospy.init_node("p3at_input")
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if dev: read_controller(dev)
		
		if command != "" or drive_vec.x != 0 or drive_vec.y != 0: 
			alive.publish(True)
			cmd.publish(drive_vec)
		if command.split("_")[0] == "ui": ui.publish(command)
		
		# publish driving mode
		if command == "mode_manual": mode.publish(False)
		elif command == "mode_automatic": mode.publish(True)
		
		if drive_vec.z == 1: drive_vec = Vector3()
		command = ""
		rate.sleep()


if __name__ == "__main__":
	try:
		dev = None
		if len(list_devices()) != 0:			
			dev = InputDevice( list_devices()[0] )
		read_input(dev)
	except rospy.ROSInterruptExeception:
		pass
