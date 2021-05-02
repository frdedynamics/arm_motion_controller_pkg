#!/usr/bin/env python

"""
Node runner for /Classes/robot_move_with_ik_class.py
"""

import rospy
from Classes.robot_move_with_ik_class import RobotCommander
from math import pi


def main(): 
	Robot = RobotCommander(rate=100, start_node=True)
	Robot.init_subscribers_and_publishers()
	# Robot.start_server()
	try:
		while not rospy.is_shutdown():
			Robot.update()
			Robot.r.sleep()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()