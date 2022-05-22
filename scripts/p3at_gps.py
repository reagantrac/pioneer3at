#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from serial import Serial #pyserial

def get_gps_point():
	try:
		ser = Serial("/dev/ttyACM0", 9600)
	except:
		return (115.8171660979887, -31.98057735055060)
	while True:
		data = ser.readline().decode('ISO-8859-1')
		if data.split(",")[0] != "$GNGLL": continue
		gps = data.split(",")[1:]
		if gps[0] == "" or gps[2] == "": return (115.8171660979887, -31.98057735055060)
		
		lat = -float(gps[0])/100
		lon = float(gps[2])/100
  
		return (lon, lat)

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
