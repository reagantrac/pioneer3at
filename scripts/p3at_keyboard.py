#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import curses

def keypress(stdscr):

	key = rospy.Publisher('/p3at/key_input', String, queue_size=1)
	rospy.init_node("p3at_keyboard")
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		curses.flushinp()
		stdscr.clear()
		keypress = stdscr.getch()
		stdscr.refresh()
		key.publish(curses.unctrl(keypress).decode('ascii'))
		rate.sleep()


def main(screen):
	curses.curs_set(0)
	try:
		keypress(screen)
	except rospy.ROSInterruptExeception:
		pass


if __name__ == "__main__":
	curses.wrapper(main)
