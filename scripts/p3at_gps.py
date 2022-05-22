#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from serial import Serial #pyserial

def read_line(port, eol="\n"):
	leneol = len(eol)
	line = bytearray()
	while True:
		line += port.read(1)
		if line[-lenol:] == eol: break
	return bytes(line)

def get_gps_point():
	
	with open("/dev/ttyACM0", "rb") as gps:
		g = gps.readline()
		while len(g) <= 2:
			data = g.decode('utf-8')
			g = gps.readline()
			if data.split(",")[0] != "$GNGLL": continue
			gps = data.split(",")[1:]
			if data.split(",")[0] == "" or data.split(",")[2] == "": return (115.8171660979887, -31.98057735055060)
			rospy.loginfo(",".join(data))
			lat = -float(data.split(",")[0])/100
			lon = float(data.split(",")[2])/100
	  
			return (lon, lat)
	return (115.8171660979887, -31.98057735055060)
def read_gps():

	g = rospy.Publisher('/p3at/gps', Vector3, queue_size=1)
	rospy.init_node("p3at_gps")
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pos = Vector3()
		lon, lat = get_gps_point()
		pos.x = lon
		pos.y = lat
		g.publish(pos)
		rate.sleep()

if __name__ == "__main__":
	try:
		read_gps()
	except rospy.ROSInterruptExeception:
		pass
