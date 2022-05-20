#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys, termios
from select import select

def set_terminal(fd, term):
	termios.tcsetattr(fd, termios.TCSAFLUSH, term)

def set_normal_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

def set_curses_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr != []

def getch():
	k = sys.stdin.read(1)
	termios.tcflush(sys.stdin, termios.TCIOFLUSH)
	return k


def keypress():

	key = rospy.Publisher('/p3at/key_input', String, queue_size=1)
	rospy.init_node("p3at_keyboard")
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		keypress = getch()
		key.publish(keypress)
		rate.sleep()


if __name__ == "__main__":
	# Save normal terminal settings
	fd = sys.stdin.fileno()
	old_term = termios.tcgetattr(fd)

	# Debuffer new terminal until exit
	new_term = termios.tcgetattr(fd)
	new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)

	set_terminal(fd, new_term)

	try:
		keypress()
	except rospy.ROSInterruptExeception:
		pass
	
	set_terminal(fd, old_term)
