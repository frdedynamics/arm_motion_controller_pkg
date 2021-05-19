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
from math import radians as d2r
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int16

import actionlib
import control_msgs.msg as cm
import trajectory_msgs.msg as tm

sys.path.append('/home/gizem/catkin_ws/src/my_human_pkg/src/Classes')
import Kinematics_with_Quaternions as kinematic

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
		# self.robot_init = Pose(Point(0.3921999999969438, 0.08119999999999986,  0.6871000000019204), Quaternion(0.0, 0.0, 0.0, 1.0)) # home = [0.0, -pi/2, pi/2, pi, -pi/2, 0.0]
		# self.robot_init = Pose(Point(-0.08119999999999973, 0.3921999999969438,  0.6871000000019204), Quaternion(0.0, 0.0, 0.707, 0.707))  # home = [pi/2, -pi/2, pi/2, pi, -pi/2, 0.0]


		self.robot_init = Pose(Point(-0.268719045144, -0.338337565315,  0.148510892571), Quaternion(-0.345444335525, 0.643054800766, 0.63727196504, -0.247048936124))  # home = [pi/2, -pi/2, pi/2, pi, -pi/2, 0.0]
		self.release_approach = Pose(Point(0.6395040721, -0.097155082343, 0.489161062743), Quaternion(-0.691030388932, 0.0919664982241, -0.0804260519973, 0.71242600664))
		self.release = Pose(Point(0.638477428288, -0.0945406788611, 0.383795746435), Quaternion(-0.690892925071, 0.0923178347331, -0.0809326119407, 0.712456522043))

		print "============ Arm current pose: ", self.robot_init
		# print "click Enter to continue"
		# dummy_input = raw_input()

		self.home = [d2r(-139.29), d2r(-39.49), d2r(117.12), d2r(109.50), d2r(-80.88), d2r(92.66)]
		self.target_pose = Pose()
		self.motion_hand_pose = Pose()
		self.hand_grip_strength = Int16()
		self.steering_hand_pose = Pose()
		self.openrave_joint_angles = JointState()
		self.openrave_joint_angles.position = self.home
		self.robot_pose = Pose()
		self.robot_joint_angles = JointState()

		self.hand_init_orientation = Quaternion()

		self.s = s
		self.k = k

		self.init_flag = False

		self.g = cm.FollowJointTrajectoryGoal()
		self.g.trajectory = tm.JointTrajectory()
		self.g.trajectory.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] # gazebo model has this interesting order of joints

		# self.states = { "1": "IDLE",
		# 				"2": "APPROACH",
		# 				"3": "LIFT",
		# 				"4": "RELEASE"
		# 			  }
		self.state = "IDLE"
		self.role = "HUMAN_LEADING"  # of "ROBOT_LEADING"
		self.hrc_status = String()
               

	def init_subscribers_and_publishers(self):
		self.sub_hand_grip_strength = rospy.Subscriber('/hand_grip_strength', Int16, self.cb_hand_grip_strength)
		self.sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, self.cb_hand_pose)
		self.sub_steering_pose = rospy.Subscriber('/steering_pose', Pose, self.cb_steering_pose)
		self.sub_openrave_joints = rospy.Subscriber('/joint_states_openrave', JointState, self.cb_openrave_joints)
		self.sub_robot_joints = rospy.Subscriber('/joint_states', JointState, self.cb_robot_joints)
		self.pub_tee_goal = rospy.Publisher('/Tee_goal_pose', Pose, queue_size=1)
		self.pub_hrc_status = rospy.Publisher('/hrc_status', String, queue_size=1)


	def cb_hand_grip_strength(self, msg):
		""" Subscribes hand grip strength
		Open: 0
		Close: 255 """
		self.hand_grip_strength = msg


	def cb_hand_pose(self, msg):
		""" Subscribes left hand pose """
		self.motion_hand_pose = msg
		if not self.init_flag:
			self.hand_init_orientation = kinematic.q_invert(self.motion_hand_pose.orientation)
			print "Hand init set:", self.hand_init_orientation
			self.init_flag = True
	

	def cb_steering_pose(self, msg):
		""" Subscribes right hand pose """
		self.steering_hand_pose = msg

	
	def cb_openrave_joints(self, msg):
		""" Subscribes calculated joint angles from IKsolver """
		self.openrave_joint_angles = msg
		# print "openrave joint angles:", self.openrave_joint_angles


	def cb_robot_joints(self, msg):
		""" Subscribes real robot angles """
		self.robot_joint_angles = msg


	def cartesian_control_2_arms(self):	
		self.target_pose.position.x = self.motion_hand_pose.position.x + self.s * self.steering_hand_pose.position.x
		self.target_pose.position.y = self.motion_hand_pose.position.y + self.s * self.steering_hand_pose.position.y
		self.target_pose.position.z = self.motion_hand_pose.position.z + self.s * self.steering_hand_pose.position.z
		self.target_pose.orientation = self.motion_hand_pose.orientation

		# print "robot_pose:", self.robot_pose.position
		self.robot_pose.position.x = self.robot_init.position.x + self.k * self.target_pose.position.x
		self.robot_pose.position.y = self.robot_init.position.y + self.k * self.target_pose.position.y
		self.robot_pose.position.z = self.robot_init.position.z + self.k * self.target_pose.position.z
		self.robot_pose.orientation = kinematic.q_multiply(self.robot_init.orientation, kinematic.q_multiply(self.hand_init_orientation, self.motion_hand_pose.orientation))
		# robot_pose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose_target.orientation)


	def cartesian_control_1_arm(self):	
		# print "robot_pose:", self.robot_pose.position
		self.robot_pose.position.x = self.robot_init.position.x + self.k * self.motion_hand_pose.position.x
		self.robot_pose.position.y = self.robot_init.position.y + self.k * self.motion_hand_pose.position.y
		self.robot_pose.position.z = self.robot_init.position.z + self.k * self.motion_hand_pose.position.z
		self.robot_pose.orientation = kinematic.q_multiply(self.robot_init.orientation, kinematic.q_multiply(self.hand_init_orientation, self.motion_hand_pose.orientation))
		# robot_pose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose_target.orientation)


	@staticmethod
	def jointcomp(joints_list1, joints_list2):
		j1 = np.array(joints_list1)
		j2 = np.array([joints_list2[2], joints_list2[1], joints_list2[0], joints_list2[3], joints_list2[4], joints_list2[5]])
		# absolute(a - b) <= (atol + rtol * absolute(b))
		print j1[0]-j2[0]
		return np.allclose(j1, j2, rtol=1e-03, atol=1e-04)

	
	def robot_move_predef_pose(self, goal):
		result = False
		if not result:
			self.pub_tee_goal.publish(goal)
			result = RobotCommander.jointcomp(self.robot_joint_angles.position, self.openrave_joint_angles.position)
			print "result", result
		return result


	def update(self):
		# Palm up: active, palm dowm: idle
		if not self.role == "ROBOT_LEADING":
			if(self.state == "CO-LIFT"):
				if(self.steering_hand_pose.position.x < -0.3 and self.steering_hand_pose.position.z < -0.4):
					self.state = "RELEASE"
				else:
					if(self.steering_hand_pose.orientation.w > 0.707 and self.steering_hand_pose.orientation.x < 0.707):
						self.state = "IDLE"
					else:
						self.state == "CO-LIFT"
			elif(self.steering_hand_pose.orientation.w > 0.707 and self.steering_hand_pose.orientation.x < 0.707):
				self.state = "IDLE"
				# if steering arm vertically downwords when it is in IDLE
				# if(self.steering_hand_pose.position.x < -0.3 and self.steering_hand_pose.position.z < -0.4):
				# 	self.state = "RELEASE"
			elif(self.steering_hand_pose.orientation.w < 0.707 and self.steering_hand_pose.orientation.x > 0.707):
				if not (self.state == "CO-LIFT"):
					self.state = "APPROACH"

			try:
				if(self.state == "APPROACH" or self.state == "CO-LIFT"): # ACTIVE
					# check grip here
					# print "self.hand_grip_strength.data:", self.hand_grip_strength.data
					if(self.hand_grip_strength.data > 100):
						self.state = "CO-LIFT"
						self.cartesian_control_1_arm()  # one hand free
						# do something extra? Change axes? Maybe robot take over from here?
						# No way to leave CO-LIFT state unless hand releases
					else:
						self.cartesian_control_2_arms()
						
					self.pub_tee_goal.publish(self.robot_pose)
				
				elif(self.state == "IDLE"):
					pass
				elif(self.state == "RELEASE"):
					self.role = "ROBOT_LEADING"
				else:
					raise AssertionError("Unknown collaboration state")

			except AssertionError as e:
				print e

		else:
			## RELEASE APPROACH
			user_input = raw_input("Move to RELEASE APPROACH pose?")
			if user_input == 'y':
				reach_flag = False
				while not reach_flag:
					reach_flag = self.robot_move_predef_pose(self.release_approach)
				print "Robot at release approach"
			else:
				sys.exit("unknown user input")

			## RELEASE (or PLACE)
			user_input = raw_input("Move to RELEASE pose?")
			if user_input == 'y':
				reach_flag = False
				while not reach_flag:
					reach_flag = self.robot_move_predef_pose(self.release)
				print "Robot at RELEASE"
				# Gripper_release()
			else:
				sys.exit("unknown user input")

			## GO BACK HOME
			user_input = raw_input("Move to INIT/HOME pose?")
			if user_input == 'y':
				reach_flag = False
				while not reach_flag:
					reach_flag = self.robot_move_predef_pose(self.release)
				print "Robot at HOME"
				print "Ready to new cycle"
				print "Please move arms such that role:HUMAN_LEADING and state:IDLE"
				user_input = raw_input("Ready to new cycle?")
				if user_input == 'y':
					reach_flag = False
					while not reach_flag:
						reach_flag = self.robot_move_predef_pose(self.robot_init)
					rospy.sleep(5)
					self.role = "HUMAN_LEADING"
					self.state = "IDLE"
		
		print "state:", self.state, "    role:", self.role
		self.hrc_status = self.state + ',' + self.role
		self.pub_hrc_status.publish(self.hrc_status)
		
		# if(self.steering_hand_pose.orientation.w < 0.707 and self.steering_hand_pose.orientation.x > 0.707): # Clutch deactive
		# 	self.pub_tee_goal.publish(self.robot_pose)


		# Horizontal home right hand:
		#   x: -0.00147650952636
		# 	y: 0.0330141547947
		# 	z: 0.000745478409468
		# 	orientation:
		# 	x: 3.49697622801e-05
		# 	y: -0.000729408027073
		# 	z: 0.0405015150963
		# 	w: 0.99917921016

		# Vertical role-change pose:
		# 	x: -0.480915422135
		# 	y: -0.00188682688672
		# 	z: -0.561566305906
		# 	orientation:
		# 	x: -0.0511205762636
		# 	y: 0.648213259194
		# 	z: 0.0576285159603
		# 	w: 0.757552117967


