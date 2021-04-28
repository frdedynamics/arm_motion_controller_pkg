#! /usr/bin/env python

"""
Subscrives two wrist poses w.r.t chest. Send Tee goal w.r.t UR5e initial pose at home.

UR5e init \base_link \tool0 TF at initial pose:
- Translation: [-0.136, 0.490, 0.687]
- Rotation: in Quaternion [-0.697, 0.005, 0.012, 0.717]
            in RPY (radian) [-1.542, 0.024, 0.010]
            in RPY (degree) [-88.374, 1.403, 0.549]
"""
import sys
import rospy
import copy

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState

hand_pose = Pose()


def cb_hand_pose(msg):
    global hand_pose
    hand_pose = msg


def cartesian_control_with_IMU(robot_init, hand_pose_target, *argv):
	scale = 3.0
	
	robot_pose = Pose()
	robot_pose.position.x = robot_init.position.x + scale * hand_pose_target.position.y
	robot_pose.position.y = robot_init.position.y + scale * hand_pose_target.position.x
	robot_pose.position.z = robot_init.position.z + scale * hand_pose_target.position.z
	robot_pose.orientation = robot_init.orientation
	# robot_pose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose_target.orientation)
	pub_tee_goal.publish(robot_pose)


def main():
	try:
		global hand_pose
		sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, cb_hand_pose)
		sub_openrave_joints = rospy.Subscriber('/joint_states_openrave', JointState, queue_size=1)
		pub_tee_goal = rospy.Publisher('/Tee_goal_pose', Pose, queue_size=1)
		rospy.init_node('robot_move_with_ik')
		robot_init = Pose(Point(-0.136, 0.490, 0.687), Quaternion(-0.697, 0.005, 0.012, 0.717))
		print "============ Arm current pose: ", robot_init
		print "click Enter to continue"
		dummy_input = raw_input()
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			cartesian_control_with_IMU(robot_init, hand_pose)
			print "hand_pose:", hand_pose
			rate.sleep()
	except KeyboardInterrupt:
		moveit_commander.roscpp_shutdown()
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()