#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
import math, time

waypoints = {
	"somewhere_along_the_way": 		{"longitude": 115.8171782781675, "latitude": -31.98038731577529},
	"maybe_a_scoreboard":      		{"longitude": 115.8171702857572, "latitude": -31.98017884452402},
	"home_sweet_home": 		  		{"longitude": 115.8171314540043, "latitude": -31.98082891705035},
	"rallying_point":  		   		{"longitude": 115.8171660979887, "latitude": -31.98057735055060},
	"let_s_have_a_little_stroll": 	{"longitude": 115.8174656945906, "latitude": -31.98081842250873},
	"supporter_united": 			{"longitude": 115.8175647311226, "latitude": -31.98041252344407},
	"coffee_time": 					{"longitude": 115.8197862220968, "latitude": -31.98052211397503},
}

is_moving = False

# waypoint variables
selected_waypoint = 0
executing_waypoint = False
waypoints_list = []
waypoints_loop = False
waypoints_progress = 0

# gps navigation variables
start = Vector3()
current_xy = Vector3()
goal_xy = Vector3()

def stop():
	global executing_waypoint, waypoints_progress, is_moving
	executing_waypoint = False
	waypoints_progress = 0
	is_moving = False

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
	global current_xy, goal_xy, start, is_moving
	global executing_waypoint, waypoints_progress, waypoints_list, selected_waypoint, waypoints_loop
	if is_moving: return
	if not executing_waypoint: return
	
	is_moving = True
	drive_cmd = rospy.Publisher("/p3at/drive_cmd", Vector3, queue_size=1)
	
	# record gps at start of route
	if start.x == 0 and start.y == 0:
		start.x = current.x
		start.y = current.y
	
	# get goal
	idx = waypoints_list[waypoints_progress]
	name = list(waypoints.keys())[idx]
	goal = Vector3(waypoints[name]["longitude"], waypoints[name]["latitude"], 0)
	
	# check if at goal
	check_dist, _, _, _ = gps_distance(current, goal)
	rospy.loginfo(check_dist)
	if int(check_dist) < 5:
		rospy.loginfo(str(check_dist) + " reached goal")
		waypoints_progress += 1
		if waypoints_loop: waypoints_progress = waypoints_progress % len(waypoints_list)
		elif waypoints_progress >= len(waypoints_list): 
			stop()
			ui_cmd(String(""))
			return
		ui_cmd(String(""))
	
	# drive 5 seconds straight, needs calibration
	start_time = time.time()
	while time.time() - start_time < 5:
		drive_cmd.publish(Vector3(0.5, 0, 0))

	_, current_bearing, sx, sy = gps_distance(start, current)
	current_xy = Vector3(sx, sy, 0)
	
	_, target_bearing, gx, gy = gps_distance(current, goal)
	goal_xy = Vector3(gx, gy, 0)
		
	#rotate to face goal, needs calibration
	ang = target_bearing - current_bearing
	start_time = time.time()
	while time.time() - start_time < 5:
		drive_cmd.publish(Vector3(0, ang/(2*math.pi), 0))
	
	is_moving = False

def is_alive(data):
	global timer
	timer = 0

def ui_cmd(data):
	global selected_waypoint, executing_waypoint, waypoints_list, waypoints_loop, waypoints_progress
	command = data.data
	
	if not executing_waypoint:
		if command == "ui_waypoint_switch": 
			selected_waypoint += 1
			selected_waypoint = selected_waypoint % len(waypoints)
		elif command == "ui_waypoint_add_remove":
			if selected_waypoint in waypoints_list:
				waypoints_list.remove(selected_waypoint)
			else: waypoints_list.append(selected_waypoint)
		elif command == "ui_waypoint_loop_toggle":
			waypoints_loop = not waypoints_loop
		elif command == "ui_waypoint_execute":
			executing_waypoint = True
	elif command == "ui_waypoint_cancel":
		stop()
	
	display = rospy.Publisher("/p3at/display/text", String, queue_size=20)
	display.publish("0_.WAYPOINT SELECTOR :: :Add/Remove, :: ")
	display.publish("1_.Will Loop = "+str(waypoints_loop))
	for key in waypoints:
		idx = list(waypoints.keys()).index(key)
		selected = False
		if executing_waypoint: selected = (idx == waypoints_list[waypoints_progress])
		else: selected = (idx == selected_waypoint)
		
		prefix = str(idx+2) + "_.[ ] "
		prefix = list(prefix)
		if idx in waypoints_list: 
			prefix[4] = str(waypoints_list.index(idx))
		if selected and executing_waypoint: prefix[1] = "r"
		elif selected: prefix[1] = "s"
		
		display.publish("".join(prefix) + key)

def manual_drive(data):
	global executing_waypoint
	if not executing_waypoint:
		drive_cmd = rospy.Publisher("/p3at/drive_cmd", Vector3, queue_size=1)
		drive_cmd.publish(Vector3(data.x, data.y, 0))

def navigation():
	rospy.init_node("p3at_navigation")
	
	rospy.Subscriber("/p3at/keep_alive", Bool, is_alive)
	rospy.Subscriber("/p3at/ui_cmd", String, ui_cmd)
	rospy.Subscriber("/p3at/manual_cmd", Vector3, manual_drive)
	rospy.Subscriber("/p3at/gps", Vector3, gps_point)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		rate.sleep()


if __name__ == "__main__":
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
