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
	def __init__(self, rate=100, start_node=False, s=1.0, k=1.0):
		"""Initializes the robot commander
			@params s: motion hand - steering hand scale
			@params k: target hand pose - robot pose scale"""

		if start_node == True:
			rospy.init_node("robot_move_with_ik")
			self.r = rospy.Rate(rate)
			print "robot_move_with_ik Node Created"

		# self.robot_init = Pose(Point(-0.136, 0.490, 0.687), Quaternion(-0.697, 0.005, 0.012, 0.717))
		self.robot_init = Pose(Point(0.3921999999969438, 0.08119999999999986,  0.6871000000019204), Quaternion(0.0, 0.0, 0.0, 1.0)) # home = [0.0, -pi/2, pi/2, pi, -pi/2, 0.0]
		# self.robot_init = Pose(Point(-0.08119999999999973, 0.3921999999969438,  0.6871000000019204), Quaternion(0.0, 0.0, 0.707, 0.707))  # home = [pi/2, -pi/2, pi/2, pi, -pi/2, 0.0]


		print "============ Arm current pose: ", self.robot_init
		# print "click Enter to continue"
		# dummy_input = raw_input()

		self.home = [0.0, -pi/2, pi/2, pi, -pi/2, 0.0]
		self.target_pose = Pose()
		self.motion_hand_pose = Pose()
		self.steering_hand_pose = Pose()
		self.joint_angles = JointState()
		self.joint_angles.position = self.home
		self.robot_pose = Pose()

		self.s = s
		self.k = k

		self.move_safe_flag = False

		self.g = cm.FollowJointTrajectoryGoal()
		self.g.trajectory = tm.JointTrajectory()
		self.g.trajectory.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] # gazebo model has this interesting order of joints
               

	def init_subscribers_and_publishers(self):
		self.sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, self.cb_hand_pose)
		self.sub_steering_pose = rospy.Subscriber('/steering_pose', Pose, self.cb_steering_pose)
		self.sub_openrave_joints = rospy.Subscriber('/joint_states_openrave', JointState, self.cb_openrave_joints)
		self.pub_tee_goal = rospy.Publisher('/Tee_goal_pose', Pose, queue_size=1)


	def cb_hand_pose(self, msg):
		""" Subscribes left hand pose """
		self.motion_hand_pose = msg
	

	def cb_steering_pose(self, msg):
		""" Subscribes right hand pose """
		self.steering_hand_pose = msg

	
	def cb_openrave_joints(self, msg):
		""" Subscribes calculated joint angles from IKsolver """
		self.joint_angles = msg
		print "openrave joint angles:", self.joint_angles


	def cartesian_control_with_IMU(self):	
		self.target_pose.position.x = self.motion_hand_pose.position.x + self.s * self.steering_hand_pose.position.x
		self.target_pose.position.y = self.motion_hand_pose.position.y + self.s * self.steering_hand_pose.position.y
		self.target_pose.position.z = self.motion_hand_pose.position.z + self.s * self.steering_hand_pose.position.z
		self.target_pose.orientation = self.motion_hand_pose.orientation

		print "robot_pose:", self.robot_pose.position
		self.robot_pose.position.x = self.robot_init.position.x + self.k * self.target_pose.position.x
		self.robot_pose.position.y = self.robot_init.position.y + self.k * self.target_pose.position.y
		self.robot_pose.position.z = self.robot_init.position.z + self.k * self.target_pose.position.z
		self.robot_pose.orientation = self.robot_init.orientation
		# robot_pose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose_target.orientation)
	

	def update(self):
		# print "here"
		self.cartesian_control_with_IMU()

		if(self.steering_hand_pose.orientation.w > 0.707 and self.steering_hand_pose.orientation.x < 0.707): # Clutch deactive
			self.pub_tee_goal.publish(self.robot_pose)
