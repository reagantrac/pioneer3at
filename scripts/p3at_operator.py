#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from pioneer3at.msg import GPS

waypoints = {
	"somewhere_along_the_way": {
		"longitude": 115.8174289881738,
		"latitude": -31.98012500998101,
		"altitude": 5.72848845264015,
		"heading": 0.6888631848299723,
		"tilt": 11.32096947939711,
		"range": 149.1389400931621
	},
}

def callback(data):
	pass

def operator():
	keep_alive = rospy.Publisher("/p3at/keep_alive", Bool, queue_size=1)
	gps = rospy.Publisher("/p3at/waypoints", GPS, queue_size=1)
	
	rospy.init_node("p3at_operator")
	
	for name in waypoints:
		g = GPS()
		print(waypoints[name])
		g.longitude = waypoints[name]["longitude"]
		g.latitude = waypoints[name]["latitude"]
		g.altitude = waypoints[name]["altitude"]
		g.heading = waypoints[name]["heading"]
		g.tilt = waypoints[name]["tilt"]
		g.range = waypoints[name]["range"] 
		gps.publish(g)
	
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		keep_alive.publish(True)
		rate.sleep()


if __name__ == "__main__":
	try:
		operator()
	except rospy.ROSInterruptException:
		pass
