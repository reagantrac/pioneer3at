#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3

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

timer = 0
linear = 0
angular = 0
automatic = False

def is_alive(data):
	global timer
	timer = 0

def ui_cmd(data):
	pass

def manual_cmd(data):
	global linear, angular, automatic
	if not automatic:
		linear = data.x
		angular = data.y

def drive_mode(data):
	pass

def navigation():
	global timer, linear, angular

	drive_cmd = rospy.Publisher("/p3at/drive_cmd", Vector3, queue_size=1)

	rospy.init_node("p3at_navigation")
	
	rospy.Subscriber("/p3at/keep_alive", Bool, is_alive)
	rospy.Subscriber("/p3at/ui_cmd", String, ui_cmd)
	rospy.Subscriber("/p3at/manual_cmd", Vector3, manual_cmd)
	rospy.Subscriber("/p3at/switch_mode", Bool, drive_mode)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		timer+=0.1
		
		d = Vector3()
		d.x = linear
		d.y = angular
		d.z = 0
		if timer < 0.4: drive_cmd.publish(d)
		rate.sleep()


if __name__ == "__main__":
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
