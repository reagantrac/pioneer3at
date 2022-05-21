#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
import math

waypoints = {
	"somewhere_along_the_way": 		{"longitude": 115.8171782781675, "latitude": -31.98038731577529},
	"maybe_a_scoreboard":      		{"longitude": 115.8171702857572, "latitude": -31.98017884452402},
	"home_sweet_home": 		  		{"longitude": 115.8171314540043, "latitude": -31.98082891705035},
	"rallying_point":  		   		{"longitude": 115.8171660979887, "latitude": -31.98057735055060},
	"let_s_have_a_little_stroll": 	{"longitude": 115.8174656945906, "latitude": -31.98081842250873},
	"supporter_united": 			{"longitude": 115.8175647311226, "latitude": -31.98041252344407},
	"coffee_time": 					{"longitude": 115.8197862220968, "latitude": -31.98052211397503},
}

timer = 0
linear = 0
angular = 0
automatic = False
position = [0.0,0.0]

selected_waypoint = 0
executing_waypoint = False
	
def gps_distance(point1, point2):
	lat1 = math.radians(point1["latitude"])
	lon1 = math.radians(point1["longitude"])
	lat2 = math.radians(point2["latitude"])
	lon2 = math.radians(point2["longitude"])
	dlat = (lat2-lat1)
	dlon = (lon2-lon1)
	a = math.sin(dlat/2)**2  + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	dist = 6371e3*c
	
	y = math.sin(dlon) * math.cos(lat2)
	x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
	ang = math.atan2(y, x)
	return dist, ang, -dist*math.cos(ang), dist*math.sin(ang)

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
	global automatic
	automatic = not automatic

def navigation():
	global timer, linear, angular

	drive_cmd = rospy.Publisher("/p3at/drive_cmd", Vector3, queue_size=1)
	
	#print(gps_distance(waypoints["maybe_a_scoreboard"], waypoints["somewhere_along_the_way"]))

	rospy.init_node("p3at_navigation")
	
	rospy.Subscriber("/p3at/keep_alive", Bool, is_alive)
	rospy.Subscriber("/p3at/ui_cmd", String, ui_cmd)
	rospy.Subscriber("/p3at/manual_cmd", Vector3, manual_cmd)
	rospy.Subscriber("/p3at/switch_mode", Bool, drive_mode)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		
		d = Vector3()
		d.x = linear
		d.y = angular
		if timer == 0: drive_cmd.publish(d)
		timer+=0.1
		rate.sleep()


if __name__ == "__main__":
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
