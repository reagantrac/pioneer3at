#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
from evdev import list_devices, InputDevice, categorize, ecodes

drive_vec = Vector3()
command = ""
alive = False

button_pressed = []

def read_controller(dev):
	global command, drive_vec, button_pressed
	stick_min = 10000
	stick_max = 53000

	event = dev.read_one()
	while event != None: 
		if event.type == ecodes.EV_KEY:
			code = event.code
			if   code == 309 and code not in button_pressed: command = "ui_waypoint_switch"
			elif code == 307 and code not in button_pressed: command = "ui_waypoint_add_remove"
			elif code == 306 and code not in button_pressed: command = "ui_waypoint_loop_toggle"
			elif code == 305 and code not in button_pressed: command = "ui_waypoint_execute"
			elif code == 304 and code not in button_pressed: command = "ui_waypoint_cancel"
			
			if event.value == 0:
				if code in button_pressed: button_pressed.remove(code)
			elif code not in button_pressed: button_pressed.append(code)
		#read stick axis movement
		elif event.type == ecodes.EV_ABS:
			if event.code == 3:
				y = event.value - (stick_max)/2 - stick_min
				y /= stick_max
				drive_vec.y = round(-y,1)
				key_pressed = "k"
			if event.code == 4:
				x = event.value - (stick_max)/2 - stick_min
				x /= stick_max
				drive_vec.x = round(-x,1)
		event = dev.read_one()
		

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
	
	if data.data == "q":   command = "ui_waypoint_switch"
	elif data.data == "e": command = "ui_waypoint_add_remove"
	elif data.data == "c": command = "ui_waypoint_loop_toggle"
	elif data.data == "x": command = "ui_waypoint_execute"
	elif data.data == "z": command = "ui_waypoint_cancel"

def read_input(dev):
	global command, drive_vec, button_pressed

	ui = rospy.Publisher("/p3at/ui_cmd", String, queue_size=3)
	cmd = rospy.Publisher("/p3at/manual_cmd", Vector3, queue_size=1)
	mode = rospy.Publisher("/p3at/switch_mode", Bool, queue_size=1)
	alive = rospy.Publisher("/p3at/keep_alive", Bool, queue_size=1)

	rospy.Subscriber('/p3at/key_input', String, keypress)
	rospy.init_node("p3at_input")
	
	alive.publish(True) # wake up the robot, one time

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if dev: read_controller(dev)
		
		if drive_vec.z == 1 and (command != "" or drive_vec.x != 0 or drive_vec.y != 0):
			alive.publish(True)
			cmd.publish(drive_vec)
		elif drive_vec.z == 0 and 311 in button_pressed:
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
