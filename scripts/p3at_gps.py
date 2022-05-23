#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from serial import Serial #pyserial
import numpy as np

running_average = [[],[],[],[],[],[],[],[],[],[]]

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

def get_gps_point():
	with open("/dev/ttyACM0", "rb") as gps:
		g = gps.readline()
		while len(g) >= 2:
			data = g.decode('utf-8')
			g = gps.readline()
			if data.split(",")[0] != "$GNGLL": continue
			if data.split(",")[0] == "" or data.split(",")[2] == "": continue
			rospy.loginfo(",".join(data))
			lat = -float(data.split(",")[0])/100
			lon = float(data.split(",")[2])/100
			
			add_gps_point(lon, lat)
			
			return np.average(running_average[running_average != []], axis=0, weights=np.linspace(1.0,2.0,0.1))
	#np.mean(running_average[running_average != []])
def read_gps():

	g = rospy.Publisher('/p3at/gps', Vector3, queue_size=1)
	rospy.init_node("p3at_gps")
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pos = Vector3()
		lon_lat = get_gps_point()
		pos.x = lon_lat[0]
		pos.y = lon_lat[1]
		g.publish(pos)
		rate.sleep()

if __name__ == "__main__":
	try:
		read_gps()
	except rospy.ROSInterruptExeception:
		pass
