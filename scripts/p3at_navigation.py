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

is_alive = False
alive_timer = 0

# waypoint variables
selected_waypoint = 0
executing_waypoint = False
waypoints_list = []
waypoints_loop = False
waypoints_progress = 0

# gps navigation variables
previous = Vector3()
current_gps = Vector3()

def stop():
	global executing_waypoint, waypoints_progress, previous
	executing_waypoint = False
	waypoints_progress = 0
	previous = Vector3()

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
	global current_gps
	current_gps = Vector3(current.x , current.y, 0)

def is_alive(data):
	global is_alive, alive_timer
	is_alive = True
	alive_timer = 0

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
			executing_waypoint = len(waypoints_list) > 0
	elif command == "ui_waypoint_cancel": stop()
	
	display = rospy.Publisher("/p3at/display/text", String, queue_size=20)
	display.publish("0_.WAYPOINT SELECTOR :: Right Bumper to switch option")
	display.publish("1_.Y :: Loop waypoints = "+str(waypoints_loop))
	display.publish("9_.Press A to start navigation / Press B to return to manual driving")
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
	global executing_waypoint, is_alive
	if not executing_waypoint and is_alive:
		drive_cmd = rospy.Publisher("/p3at/drive_cmd", Vector3, queue_size=1)
		drive_cmd.publish(Vector3(data.x, data.y, 0))

def navigation():
	global current_gps, previous, is_alive, alive_timer
	global executing_waypoint, waypoints_progress, waypoints_list, selected_waypoint, waypoints_loop
	rospy.init_node("p3at_navigation")
	
	rospy.Subscriber("/p3at/keep_alive", Bool, is_alive)
	rospy.Subscriber("/p3at/ui_cmd", String, ui_cmd)
	rospy.Subscriber("/p3at/manual_cmd", Vector3, manual_drive)
	rospy.Subscriber("/p3at/gps", Vector3, gps_point)
	
	ui_cmd(String("")) # update screen
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
        # check if deadman's switch is pressed
		alive_timer += 0.1
		if alive_timer > 0.3: is_alive = False
		if is_alive and executing_waypoint and len(waypoints_list) != 0:
			if previous.x == 0 and previous.y == 0: previous = current_gps
			
            # get current goal gps point
			idx = waypoints_list[waypoints_progress]
			name = list(waypoints.keys())[idx]
			goal = Vector3(waypoints[name]["longitude"], waypoints[name]["latitude"], 0)
			
			gd, ga, gx, gy = gps_distance(current_gps, goal)
			sd, sa, sx, sy = gps_distance(previous, current_gps)
			
            # if at goal go to next point
			if int(gd) < 5:	
				waypoints_progress += 1
				
				if waypoints_loop: waypoints_progress = waypoints_progress % len(waypoints_list)
				elif waypoints_progress >= len(waypoints_list): 
					stop()
					ui_cmd(String(""))
					return
				ui_cmd(String(""))
				idx = waypoints_list[waypoints_progress]
				name = list(waypoints.keys())[idx]
				goal = Vector3(waypoints[name]["longitude"], waypoints[name]["latitude"], 0)
			
			drive_cmd = rospy.Publisher("/p3at/drive_cmd", Vector3, queue_size=1)
			log = rospy.Publisher("/p3at/log_ang", Vector3, queue_size=10)
			
			# drive forward
			start_time = time.time()
			while time.time() - start_time < 12:
				drive_cmd.publish(Vector3(0.2, 0, 0))
				if not executing_waypoint or not is_alive: break
			
			# re-evaluate goal direction
			gd, ga, gx, gy = gps_distance(current_gps, goal)
			sd, sa, sx, sy = gps_distance(previous, current_gps)
			ang = ga - sa
			deg = math.degrees(ang) % 360
			deg = (deg + 360) % 360
			if deg > 180: deg -=360
			log.publish(Vector3(math.degrees(ga), math.degrees(sa), deg))
			log.publish(Vector3(gd, sd, 0))
			
			# turn on the spot towards the goal
			start_time = time.time()
			if deg > 10 or deg < -10:
				dur = abs(deg / 30)
				while time.time() - start_time < dur:
					if not executing_waypoint or not is_alive: break
					if deg < -10: turn = Vector3(0, -0.3, 0)
					elif deg > 10: turn = Vector3(0, 0.3, 0)
					drive_cmd.publish(turn)
			
			previous = current_gps
		rate.sleep()

if __name__ == "__main__":
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
