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
	[115.8171314540043,-31.98082891705035],
]

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

def get_gps_point():
	global f
	

		
	for bline in f:
		line = bline.decode("ISO-8859-1")
 
		g = line.split(",")
		#arr = np.array(running_average)
		if g[0] != "$GNGLL": continue

		#log = rospy.Publisher("/p3at/log_gps", String, queue_size=1)
		#log.publish(line)
		
		
		if g[1] == "" or g[3] == "": break
		lat = -float(g[1])/100
		lon = float(g[3])/100
		
		add_gps_point(lon, lat)
		break
		#arr = running_average[running_average != []
	arr = np.array(running_average)
	#f.seek(0,2)
		
	return np.average(arr, axis=0)
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
		log = rospy.Publisher("/p3at/log_gps", Vector3, queue_size=1)
		log.publish(pos)
		g.publish(pos)
		rate.sleep()

if __name__ == "__main__":
	try:
		read_gps()
	except rospy.ROSInterruptExeception:
		pass
