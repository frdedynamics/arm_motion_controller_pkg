#!/usr/bin/env python

from math import pi

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState


MULTIPLIER = 1.0
SHIFT = 0.0
SECONDS_PER_MOVE = 0.0010  # doesn't need to be accurate, will accelerate to catch up and smooth its movement


NODE_NAME = "ur5e_sin_publisher"
PUBLISHER = "/arm_controller/command"
SUBSCRIBER = "/joint_states_openrave"

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# dont edit below pls
NSECS = SECONDS_PER_MOVE % 1 * 10000000000
SECS = int(SECONDS_PER_MOVE)
home = [0.0, -pi/2, pi/2, pi, -pi/2, 0.0]


def get_publisher_msg(openrave_joints):
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = JOINT_NAMES
    joint_trajectory_point = JointTrajectoryPoint()
    # joint_trajectory_point.positions = [1.57, -1.57, -3.14, -1.57, rotation * MULTIPLIER + SHIFT, 0.0]
    joint_trajectory_point.positions = openrave_joints
    joint_trajectory_point.time_from_start.secs = SECS
    joint_trajectory_point.time_from_start.nsecs = NSECS
    msg.points.append(joint_trajectory_point)
    return msg

def move_home():
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = JOINT_NAMES
    joint_trajectory_point = JointTrajectoryPoint()
    # joint_trajectory_point.positions = [1.57, -1.57, -3.14, -1.57, rotation * MULTIPLIER + SHIFT, 0.0]
    joint_trajectory_point.positions = home
    joint_trajectory_point.time_from_start.secs = SECS
    joint_trajectory_point.time_from_start.nsecs = NSECS
    msg.points.append(joint_trajectory_point)
    return msg

joint_sent = Vector3

class Spinner:
    def __init__(self):
        self.pub = rospy.Publisher(PUBLISHER, JointTrajectory, queue_size=1)
        self.sub = rospy.Subscriber(SUBSCRIBER, JointState, self.callback_openrave)

    def callback_openrave(self, msg):
        joint_sent = get_publisher_msg(msg.position)
	# print "sent:", joint_sent
        self.pub.publish(joint_sent)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    Spinner()
    move_home()
    rospy.loginfo("Initialized.")
    rospy.spin()