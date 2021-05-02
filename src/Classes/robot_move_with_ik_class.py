#! /usr/bin/env python

"""
Subscribes one wrist pose (for now) w.r.t chest. Move robot w.r.t UR5e initial pose at home.

UR5e init \base_link \tool0 TF at initial pose:
- Translation: [-0.136, 0.490, 0.687]
- Rotation: in Quaternion [-0.697, 0.005, 0.012, 0.717]
            in RPY (radian) [-1.542, 0.024, 0.010]
            in RPY (degree) [-88.374, 1.403, 0.549]
"""
import sys
import rospy
import copy
from math import pi

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState

import actionlib
import control_msgs.msg as cm
import trajectory_msgs.msg as tm

class RobotCommander:
	def __init__(self, rate=100, start_node=False, scale=1.0):
		"""Initializes the robot commander"""

		if start_node == True:
			rospy.init_node("robot_move_with_ik")
			self.r = rospy.Rate(rate)
			print "robot_move_with_ik Node Created"

		self.robot_init = Pose(Point(-0.136, 0.490, 0.687), Quaternion(-0.697, 0.005, 0.012, 0.717))
		print "============ Arm current pose: ", self.robot_init
		# print "click Enter to continue"
		# dummy_input = raw_input()

		self.home = [pi/2, -pi/2, 0.0, pi, -pi/2, 0.0]
		self.target_pose = Pose()
		self.joint_angles = JointState()
		self.joint_angles.position = self.home
		self.robot_pose = Pose()
		self.scale = scale
		self.move_safe_flag = False

		self.g = cm.FollowJointTrajectoryGoal()
		self.g.trajectory = tm.JointTrajectory()
		self.g.trajectory.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] # gazebo model has this interesting order of joints
               

	def init_subscribers_and_publishers(self):
		self.sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, self.cb_target_pose)
		self.sub_openrave_joints = rospy.Subscriber('/joint_states_openrave', JointState, self.cb_openrave_joints)
		self.pub_tee_goal = rospy.Publisher('/Tee_goal_pose', Pose, queue_size=1)


	def cb_target_pose(self, msg):
		""" Subscribes hand pose """
		print "cb_target_pose"
		self.target_pose = msg
		print "target pose:", self.target_pose

	
	def cb_openrave_joints(self, msg):
		""" Subscribes calculated joint angles from IKsolver """
		self.joint_angles = msg
		print "openrave joint angles:", self.joint_angles


	def cartesian_control_with_IMU(self):	
		print "robot_pose:", self.robot_pose.position
		self.robot_pose.position.x = self.robot_init.position.x + self.scale * self.target_pose.position.y
		self.robot_pose.position.y = self.robot_init.position.y + self.scale * self.target_pose.position.x
		self.robot_pose.position.z = self.robot_init.position.z + self.scale * self.target_pose.position.z
		self.robot_pose.orientation = self.robot_init.orientation
		# robot_pose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose_target.orientation)
	

	def update(self):
		# print "here"
		# test = [pi/2, -pi/2, 0.0, pi, pi/2, 0.0]
		# self.send_joint_commands(test)
		self.cartesian_control_with_IMU()
		# self.send_joint_commands(self.joint_angles.position)
		self.pub_tee_goal.publish(self.robot_pose)


	# def start_server(self):
	# 	self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', cm.FollowJointTrajectoryAction)
	# 	print "Waiting for server..."
	# 	self.client.wait_for_server()
	# 	print "Connected to server"
	# 	print "Please make sure that your robot can move freely. Moving home."
	# 	print "Press Enter to proceed: (Type 'n' for not safe)"
	# 	dummy_input = raw_input()
	# 	if not dummy_input == 'n':
	# 		self.move_safe_flag = True

	# 	if self.move_safe_flag == True:
	# 		self.send_joint_commands(self.home)


	# def send_joint_commands(self, current):
	# 	joint_states = rospy.wait_for_message("joint_states", JointState)
	# 	joints_pos = joint_states.position
	# 	self.g.trajectory.points = [
	# 		tm.JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
	# 		tm.JointTrajectoryPoint(positions=current, velocities=[0]*6, time_from_start=rospy.Duration(0.4))]
	# 	self.client.send_goal(self.g)
	# 	self.client.wait_for_result()
