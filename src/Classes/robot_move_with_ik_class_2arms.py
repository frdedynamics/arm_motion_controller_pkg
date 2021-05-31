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
from std_msgs.msg import String, Int16, Float64, Bool

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

		self.robot_init = Pose(Point(0.0541860145827, -0.584139173043,  0.189550620537), Quaternion(0.542714478609, -0.464001894512, -0.516758121814, 0.472360328708))  # home = [pi/2, -pi/2, pi/2, pi, -pi/2, 0.0]
		self.release_approach = Pose(Point(-0.465337306348, -0.226618254474, 0.0329134063267), Quaternion(0.561379785159, -0.440200301152, -0.496712122833, 0.494321250515))
		self.release = Pose(Point(-0.460710112314, -0.353837082507, 0.0171878089798), Quaternion(0.561713498746, -0.4400991731221, -0.496436115888, 0.494309463783))

		print "============ Arm current pose: ", self.robot_init
		# print "click Enter to continue"
		# dummy_input = raw_input()

		self.home = [-1.61718929, -0.90831965,  1.47090709, -0.43931657,  1.48942041, -1.56104302]
		self.target_pose = Pose()
		self.motion_hand_pose = Pose()
		self.hand_grip_strength = Int16()
		self.steering_hand_pose = Pose()
		self.openrave_joint_angles = JointState()
		self.openrave_joint_angles.position = self.home
		self.robot_pose = Pose()
		self.robot_joint_angles = JointState()

		self.tee_calc = Pose()

		self.robot_colift_init = Pose()
		self.target_pose_colift_init = Pose()
		self.motion_hand_colift_init = Pose()
		self.motion_hand_colift_pos_ch = Point()

		self.hand_init_orientation = Quaternion()
		self.human_to_robot_init_orientation = Quaternion(0.0, 0.0, 0.707, 0.707)

		self.s = s
		self.k = k

		self.init_flag = False
		self.colift_flag = False
		self.joint_flag = False

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
		self.sub_hand_grip_strength = rospy.Subscriber('/robotiq_grip_gap', Int16, self.cb_hand_grip_strength)
		self.sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, self.cb_hand_pose)
		self.sub_steering_pose = rospy.Subscriber('/steering_pose', Pose, self.cb_steering_pose)
		self.sub_tee_calc = rospy.Subscriber('/Tee_calculated', Pose, self.cb_tee_calc)
		self.sub_openrave_joints = rospy.Subscriber('/joint_states_openrave', JointState, self.cb_openrave_joints)
		self.sub_robot_joints = rospy.Subscriber('/joint_states', JointState, self.cb_robot_joints)
		self.pub_tee_goal = rospy.Publisher('/Tee_goal_pose', Pose, queue_size=1)
		self.pub_hrc_status = rospy.Publisher('/hrc_status', String, queue_size=1)
		self.pub_grip_cmd = rospy.Publisher('/cmd_grip', Bool, queue_size=1)


	def cb_hand_grip_strength(self, msg):
		""" Subscribes hand grip strength
		Open: 0
		Close: 255 """
		self.hand_grip_strength = msg


	def cb_hand_pose(self, msg):
		""" Subscribes left hand pose """
		self.motion_hand_pose = msg
		if not self.init_flag:
			self.hand_init_orientation = kinematic.q_invert(self.steering_hand_pose.orientation)
			print "Hand init set:", self.hand_init_orientation
			self.init_flag = True
	

	def cb_steering_pose(self, msg):
		""" Subscribes right hand pose """
		self.steering_hand_pose = msg

	def cb_tee_calc(self, msg):
		""" Subscribes openrave calculated pose """
		self.tee_calc = msg

	
	def cb_openrave_joints(self, msg):
		""" Subscribes calculated joint angles from IKsolver """
		self.openrave_joint_angles = msg
		# print "openrave joint angles:", self.openrave_joint_angles


	def cb_robot_joints(self, msg):
		""" Subscribes real robot angles """
		self.robot_joint_angles = msg
		self.joint_flag = True
		# print "robot joints:", self.robot_joint_angles.position


	def cartesian_control_2_arms(self):	
		self.target_pose.position.x = self.motion_hand_pose.position.x + self.s * self.steering_hand_pose.position.x
		self.target_pose.position.y = self.motion_hand_pose.position.y - self.s * self.steering_hand_pose.position.y
		self.target_pose.position.z = self.motion_hand_pose.position.z + self.s * self.steering_hand_pose.position.z
		self.target_pose.orientation = self.motion_hand_pose.orientation

		# print "robot_pose:", self.robot_pose.position
		corrected_target_pose = kinematic.q_rotate(self.human_to_robot_init_orientation, self.target_pose.position)
		self.robot_pose.position.x = self.robot_init.position.x + self.k * corrected_target_pose[0]
		self.robot_pose.position.y = self.robot_init.position.y - self.k * corrected_target_pose[1]
		self.robot_pose.position.z = self.robot_init.position.z + self.k * corrected_target_pose[2]
		# self.robot_pose.orientation = kinematic.q_multiply(self.robot_init.orientation, kinematic.q_multiply(self.hand_init_orientation, self.motion_hand_pose.orientation))
		self.robot_pose.orientation = self.robot_init.orientation

		self.motion_hand_colift_init = self.motion_hand_pose


	def cartesian_control_1_arm(self):	
		self.motion_hand_colift_pos_ch.x = self.motion_hand_pose.position.x - self.motion_hand_colift_init.position.x
		self.motion_hand_colift_pos_ch.y = self.motion_hand_pose.position.y - self.motion_hand_colift_init.position.y
		self.motion_hand_colift_pos_ch.z = self.motion_hand_pose.position.z - self.motion_hand_colift_init.position.z
		# print "self.robot_colift_init:", self.robot_colift_init
		# print "self.motion_hand_colift_pos_ch:", self.motion_hand_colift_pos_ch

		corrected_motion_hand_pose = kinematic.q_rotate(self.human_to_robot_init_orientation, self.motion_hand_colift_pos_ch)
		# print "corrected_motion_hand_pose:", corrected_motion_hand_pose
		
		# self.robot_pose.position.x = self.robot_colift_init.position.x + self.k * self.motion_hand_colift_pos_ch.x
		# self.robot_pose.position.y = self.robot_colift_init.position.y + self.k * self.motion_hand_colift_pos_ch.y
		# self.robot_pose.position.z = self.robot_colift_init.position.z + self.k * self.motion_hand_colift_pos_ch.z

		self.robot_pose.position.x = self.robot_colift_init.position.x + self.k * corrected_motion_hand_pose[0]
		self.robot_pose.position.y = self.robot_colift_init.position.y - self.k * corrected_motion_hand_pose[1]
		self.robot_pose.position.z = self.robot_colift_init.position.z + self.k * corrected_motion_hand_pose[2]
		# self.robot_pose.orientation = kinematic.q_multiply(self.robot_init.orientation, kinematic.q_multiply(self.hand_init_orientation, self.motion_hand_pose.orientation))
		self.robot_pose.orientation = self.robot_colift_init.orientation


	@staticmethod
	def jointcomp(joints_list1, joints_list2):
		j1 = np.array(joints_list1)
		j2 = np.array([joints_list2[2], joints_list2[1], joints_list2[0], joints_list2[3], joints_list2[4], joints_list2[5]])
		# absolute(a - b) <= (atol + rtol * absolute(b))
		diff = ((j1[0]-j2[0])+(j1[1]-j2[1])+(j1[2]-j2[2])+(j1[3]-j2[3])+(j1[4]-j2[4])+(j1[5]-j2[5]))
		# return np.allclose(j1, j2, rtol=1e-03, atol=1e-04)
		return diff

	
	def robot_move_predef_pose(self, goal):
		result = False
		if not result:
			self.pub_tee_goal.publish(goal)
			rospy.sleep(0.5)
			print self.robot_joint_angles.position, "current joints"
			print self.openrave_joint_angles.position, "openrave joints"
			result = RobotCommander.jointcomp(self.robot_joint_angles.position, self.openrave_joint_angles.position)
			print "result", result
		return result

	def update(self):
		global robot_colift_init
		# Palm up: active, palm dowm: idle
		if not self.role == "ROBOT_LEADING":
			if(self.state == "CO-LIFT"):
				print "steering_hand_pose.position.x and steering_hand_pose.position.z", self.steering_hand_pose.position.x, self.steering_hand_pose.position.z
				if(self.steering_hand_pose.position.x < -0.3 and self.steering_hand_pose.position.z < -0.2):
					self.state = "RELEASE"
				else:
					if(self.steering_hand_pose.orientation.w > 0.707 and self.steering_hand_pose.orientation.x < 0.707):
						self.state = "IDLE"
					else:
						self.state == "CO-LIFT"
						if not self.colift_flag:
							self.robot_colift_init = self.tee_calc
							self.colift_flag = True
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
					# print "motion:", self.motion_hand_pose.position, "hands:", self.target_pose.position
					print "self.hand_grip_strength.data:", self.hand_grip_strength.data
					if(self.hand_grip_strength.data > 75):
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
			## RELEASE (or PLACE)
			# user_input = raw_input("Move to RELEASE pose?")
			# if user_input == 'y':
			print "Move to RELEASE pose?"
			reach_dist = 10
			while abs(reach_dist)> 0.001:
				reach_dist = self.robot_move_predef_pose(self.release)
				print "moving to release. dist:", reach_dist
			cmd_release = Bool()
			cmd_release = True
			self.pub_grip_cmd.publish(cmd_release)
			print "Robot at RELEASE"
			# Gripper_release()
			# else:
			# 	sys.exit("unknown user input")

			# ## RELEASE APPROACH
			rospy.sleep(4)  # Wait until the gripper is fully open
			# user_input = raw_input("Move to RELEASE APPROACH pose?")
			# if user_input == 'y':
			reach_dist = 10
			while abs(reach_dist)> 0.001:
				reach_dist = self.robot_move_predef_pose(self.release_approach)
			print "Robot at release approach"
			# else:
			# 	sys.exit("unknown user input")

			## GO BACK HOME
			# user_input = raw_input("Move to INIT/HOME pose?")
			# if user_input == 'y':
			print "Please move arms such that role:HUMAN_LEADING and state:IDLE"
			user_input = raw_input("Ready to new cycle?")
			if user_input == 'y':
				reach_dist = 10
				while abs(reach_dist)> 0.001:
					reach_dist = self.robot_move_predef_pose(self.robot_init)
					print "moving to release. dist:", reach_dist
				# rospy.sleep(5)
				self.role = "HUMAN_LEADING"
				self.state = "IDLE"
		
		print "state:", self.state, "    role:", self.role
		# print self.robot_joint_angles.position, "current joints"
		# print self.openrave_joint_angles.position, "openrave joints"
		# if self.joint_flag:
		# 	result = RobotCommander.jointcomp(self.robot_joint_angles.position, self.openrave_joint_angles.position)
		# 	print "HERE", result
		self.hrc_status = self.state + ',' + self.role
		self.pub_hrc_status.publish(self.hrc_status)
		self.r.sleep()
		
		# if(self.steering_hand_pose.orientation.w < 0.707 and self.steering_hand_pose.orientation.x > 0.707): # Clutch deactive
		# 	self.pub_tee_goal.publish(self.robot_pose)


