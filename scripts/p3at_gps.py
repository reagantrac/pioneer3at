#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from serial import Serial #pyserial
import numpy as np

running_average = [
	[115.8171314540043,-31.98082891705035],
	[115.8171314540043,-31.98082891705035],
	[115.8171314540043,-31.98082891705035],
	[115.8171314540043,-31.98082891705035],
	[115.8171314540043,-31.98082891705035],
	[115.8171314540043,-31.98082891705035],
	[115.8171314540043,-31.98082891705035],
	[115.8171314540043,-31.98082891705035],
	[115.8171314540043,-31.98082891705035],
]

avg_heading = [0, 0, 0, 0, 0,]

f = open("/dev/ttyACM0", "rb")

def read_line(port, eol="\n"):
	leneol = len(eol)
	line = bytearray()
	while True:
		line += port.read(1)
		if line[-lenol:] == eol: break
	return bytes(line)
	
def add_gps_point(lon, lat):
	running_average.pop(0)
	running_average.append([lon, lat])


def add_heading(ang):
	avg_heading.pop(0)
	avg_heading.append(ang)

def get_gps_point():
	global f
	

		
	for bline in f:
		line = bline.decode("ISO-8859-1")
 
		g = line.split(",")
		#arr = np.array(running_average)
		if g[0] == "$GNGLL":
			if g[1] == "" or g[3] == "": break
			lat = -float(g[1])/100
			lon = float(g[3])/100
			
			add_gps_point(lon, lat)
			break
		if g[0] == "$GPVTG":
			if g[1] == "": break
			ang = float(g[1])
			
			add_heading(ang)
			break

	arr = np.array(running_average)
		
	return np.average(arr, axis=0), np.average(avg_heading)
def read_gps():

	g = rospy.Publisher('/p3at/gps', Vector3, queue_size=10)
	rospy.init_node("p3at_gps")
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pos = Vector3()
		lon_lat, ang = get_gps_point()
		pos.x = lon_lat[0]
		pos.y = lon_lat[1]
		pos.z = ang
		log = rospy.Publisher("/p3at/log_gps", Vector3, queue_size=1)
		log.publish(pos)
		g.publish(pos)
		rate.sleep()

if __name__ == "__main__":
	try:
		read_gps()
	except rospy.ROSInterruptExeception:
		pass
