#!/usr/bin/env python

import rospy, curses
from std_msgs.msg import String

stdscr = None

def ui_map(data):
	pass

def ui_text(data):
	global stdscr
	stdscr.erase()
	for i, line in enumerate(data.data.split("\n")):
		if line.split(".")[0] == "r": 
			stdscr.addstr(i, 0, line[2:], curses.A_BOLD)
			stdscr.addstr(i, 30, "[ running... ]", curses.A_BLINK)
		elif line.split(".")[0] == "s":
			stdscr.addstr(i, 0, line[2:], curses.A_BOLD)
			stdscr.addstr(i, 30, "[ENTER to run]", curses.A_BOLD)
		else: stdscr.addstr(i, 0, line[2:])	
	stdscr.refresh()

def display(screen):
	global stdscr
	stdscr = screen
	stdscr.refresh()
	curses.start_color()
	curses.curs_set(0)

	rospy.init_node("p3at_navigation")
	
	rospy.Subscriber("/p3at/display/map", String, ui_map)
	rospy.Subscriber("/p3at/display/text", String, ui_text)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown(): rate.sleep()


if __name__ == "__main__":
	curses.wrapper(display)
