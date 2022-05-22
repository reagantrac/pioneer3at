#!/usr/bin/env python

import rospy, curses
from std_msgs.msg import String

stdscr = None

def ui_map(data):
	pass

def ui_text(line):
	global stdscr
	
	line_num = int(line.data[0])
	line_mode = line.data[1]
	
	stdscr.move(line_num, 0)
	stdscr.clrtoeol()
	stdscr.move(0, 0)
	
	if line_mode == "_":
		stdscr.addstr(line_num, 0, line.data[3:])
	elif line_mode == "r":
		stdscr.addstr(line_num, 0, line.data[3:], curses.A_BOLD)
		stdscr.addstr(line_num, 40, "[ running... ]", curses.A_BLINK)
	elif line_mode == "s":
		stdscr.addstr(line_num, 0, line.data[3:], curses.A_BOLD)
		stdscr.addstr(line_num, 40, "[  X to add  ]", curses.A_BOLD)

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
