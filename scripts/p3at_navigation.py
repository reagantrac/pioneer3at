#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3

waypoints = {
	"somewhere_along_the_way": {
		"longitude": 115.8174289881738,
		"latitude": -31.98012500998101,
		"altitude": 5.72848845264015,
		"heading": 0.6888631848299723,
		"tilt": 11.32096947939711,
		"range": 149.1389400931621
	},
	"maybe_a_scoreboard": {
		"longitude": 115.8168865506287,
		"latitude": -31.98036594019854,
		"altitude": 12.24615573627041,
		"heading": -103.1012512137992,
		"tilt": 65.86535867124761,
		"range": 107.3211504674873
	},
	"home_sweet_home": {
		"longitude": 115.8172200827809,
		"latitude": -31.98100166177344,
		"altitude": 20.70200454005209,
		"heading": -17.917006228234,
		"tilt": 27.10139975787505,
		"range": 76.51809774075446
	},
}

timer = 0
linear = 0
angular = 0
automatic = False
coord = [0.0,0.0]

selected_waypoint = 0
executing_waypoint = False

def distance_to_waypoint(gps1, gps2):
	pass

def is_alive(data):
	global timer
	timer = 0

def ui_cmd(data):
	global selected_waypoint, executing_waypoint
	if data.data == "q": selected_waypoint -= 1
	elif data.data == "e": selected_waypoint += 1
	elif data.data == "^J": executing_waypoint = True
	else: return
	selected_waypoint = selected_waypoint % len(waypoints)
	
	string = ""
	for key in waypoints:
		selected = list(waypoints.keys()).index(key) == selected_waypoint
		if selected and executing_waypoint: string += "r."
		elif selected: string += "s."
		else: string += "_."
		string += key + "\n"
	
	display = rospy.Publisher("/p3at/display/text", String, queue_size=1)
	display.publish(string)

def manual_cmd(data):
	global linear, angular, automatic
	if not automatic:
		linear = data.x
		angular = data.y

def drive_mode(data):
	pass

def navigation():
	global timer, linear, angular

	drive_cmd = rospy.Publisher("/p3at/drive_cmd", Vector3, queue_size=1)
	

	rospy.init_node("p3at_navigation")
	
	rospy.Subscriber("/p3at/keep_alive", Bool, is_alive)
	rospy.Subscriber("/p3at/ui_cmd", String, ui_cmd)
	rospy.Subscriber("/p3at/manual_cmd", Vector3, manual_cmd)
	rospy.Subscriber("/p3at/switch_mode", Bool, drive_mode)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		timer+=0.1
		
		d = Vector3()
		d.x = linear
		d.y = angular
		d.z = 0
		if timer < 0.4: drive_cmd.publish(d)
		rate.sleep()


if __name__ == "__main__":
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
