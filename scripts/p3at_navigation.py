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
positions = [0.0,0.0]

drive_dist = 0
target_dist = -1

selected_waypoint = 0
executing_waypoint = False
start = Vector3()
current_xy = Vector3()
goal_xy = Vector3()
	
def gps_distance(point1, point2):
	lat1 = math.radians(point1.y)
	lon1 = math.radians(point1.x)
	lat2 = math.radians(point2.y)
	lon2 = math.radians(point2.x)
	dlat = (lat2-lat1)
	dlon = (lon2-lon1)
	a = math.sin(dlat/2)**2  + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	dist = 6371e3*c
	
	y = math.sin(dlon) * math.cos(lat2)
	x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
	ang = math.atan2(y, x)
	return dist, ang, -dist*math.cos(ang), dist*math.sin(ang)

def gps_point(current):
	global linear, angular, drive_dist, target_dist, current_xy, goal_xy, start, automatic
	if not automatic: return
	
	if drive_dist < target_dist:
		# keep driving forward
		
		linear = 0.5

	name = list(waypoints.keys())[selected_waypoint]
	goal = Vector3()
	goal.x = waypoints[name]["longitude"]
	goal.y = waypoints[name]["latitude"]

	if start.x == 0 and start.y == 0:
		start.x = current.x
		start.y = current.y
		gd, ga, gx, gy = gps_distance(current, goal)
		target_dist = gd/20
		
	
	
	sd, sa, sx, sy = gps_distance(start, current)
	current_xy.x = sx
	current_xy.y = sy
	gd, ga, gx, gy = gps_distance(current, goal)
	goal_xy.x = gx
	goal_xy.y = gy
	
	
	#rotate to face goal
	ang = math.atan2(goal_xy.y-current_xy.y, goal_xy.x-current_xy.x)
	angular = ang
		

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
	global timer, linear, angular, drive_dist

	drive_cmd = rospy.Publisher("/p3at/drive_cmd", Vector3, queue_size=1)
	
	#print(gps_distance(waypoints["maybe_a_scoreboard"], waypoints["somewhere_along_the_way"]))

	rospy.init_node("p3at_navigation")
	
	rospy.Subscriber("/p3at/keep_alive", Bool, is_alive)
	rospy.Subscriber("/p3at/ui_cmd", String, ui_cmd)
	rospy.Subscriber("/p3at/manual_cmd", Vector3, manual_cmd)
	rospy.Subscriber("/p3at/switch_mode", Bool, drive_mode)
	
	rospy.Subscriber("/p3at/gps", Vector3, gps_point)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		
		d = Vector3()
		d.x = linear
		d.y = angular
		if timer == 0: 
			drive_cmd.publish(d)
			drive_dist += 1
		timer+=0.1
		rate.sleep()


if __name__ == "__main__":
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
