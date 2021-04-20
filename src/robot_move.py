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
import moveit_commander
import copy

from geometry_msgs.msg import Pose

tee_goal = Pose()


def cb_tee_goal(msg):
    global tee_goal
    tee_goal = msg


def movegroup_init():
	"""
	Initializes the manipulator and end-effector groups
	@returns Initialized groups
	"""
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()

	arm_group = moveit_commander.MoveGroupCommander("arm")
	arm_group.set_named_target("home2")
	plan_arm = arm_group.go()  
	return arm_group

def plan_task_space_control(arm_group, robot_init, hand_pose):
	waypoints = []
	scale = 1.0
	
	tpose = Pose()
	tpose.position.x = robot_init.position.x + scale * hand_pose.position.y
	tpose.position.y = robot_init.position.y + scale * hand_pose.position.z
	tpose.position.z = robot_init.position.z + scale * hand_pose.position.x
	tpose.orientation = robot_init.orientation
	
	waypoints.append(copy.deepcopy(tpose))
	
	(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

	# arm_group.execute(plan, wait=True)
	# arm_group.stop()
	# arm_group.clear_pose_targets()

	return tpose



if __name__ == '__main__':
	global tee_goal
	sub_tee_goal = rospy.Subscriber('/tee_goal', Pose, cb_tee_goal)
	arm_group = movegroup_init()
	rospy.init_node('robot_move')
	robot_init = arm_group.get_current_pose().pose
	print "============ Arm current pose: ", robot_init
	print "click Enter to continue"
	dummy_input = raw_input()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		# tee_goal = plan_task_space_control(arm_group, robot_init, tee_goal)
		arm_group.set_pose_target(tee_goal)
		plan = arm_group.go(wait=True)
		arm_group.stop()
		arm_group.clear_pose_targets()

		print "tee_goal:", tee_goal
		rate.sleep()

