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

from geometry_msgs.msg import Pose

hand_pose = Pose()


def cb_hand_pose(msg):
    global hand_pose
    hand_pose = msg


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

def cartesian_control_with_IMU(arm_group, robot_init, hand_pose_target, *argv):
	waypoints = []
	scale = 3.0
	
	wpose = Pose()
	wpose.position.x = robot_init.position.x + scale * hand_pose_target.position.y
	wpose.position.y = robot_init.position.y + scale * hand_pose_target.position.x
	wpose.position.z = robot_init.position.z + scale * hand_pose_target.position.z
	wpose.orientation = robot_init.orientation
	# wpose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose_target.orientation)
	
	waypoints.append(copy.deepcopy(wpose))
	
	(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.1,        # eef_step
                                   0.0)         # jump_threshold

	arm_group.execute(plan, wait=False)
	arm_group.stop()
	arm_group.clear_pose_targets()



def main():
	try:
		global hand_pose
		sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, cb_hand_pose)
		arm_group = movegroup_init()
		rospy.init_node('robot_move')
		robot_init = arm_group.get_current_pose().pose
		print "============ Arm current pose: ", robot_init
		print "click Enter to continue"
		dummy_input = raw_input()
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			cartesian_control_with_IMU(arm_group, robot_init, hand_pose)
			# arm_group.set_pose_target(hand_pose)
			# plan = arm_group.go(wait=True)
			# arm_group.stop()
			# arm_group.clear_pose_targets()

			print "hand_pose:", hand_pose
			rate.sleep()
	except KeyboardInterrupt:
		moveit_commander.roscpp_shutdown()
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()