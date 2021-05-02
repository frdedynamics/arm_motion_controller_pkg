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
import math
import numpy as np

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/Classes")
from DH_matrices import DHmatrices
import Kinematics_with_Quaternions as kinematic

DHmatrices = DHmatrices()

wrist_left_pose = Pose()
wrist_right_pose = Pose()
# ur5e_init = Pose(Point(-0.136, 0.490, 0.687), Quaternion(-0.697, 0.005, 0.012, 0.717))
ur5e_init = Pose(Point(-0.492, -0.133, 0.687), Quaternion(-0.500, -0.500, 0.500, 0.500))
ur5e_init_htm = DHmatrices.pose_to_htm(ur5e_init)


def cb_l_wrist(msg):
    global wrist_left_pose
    wrist_left_pose = msg


def cb_r_wrist(msg):
    global wrist_right_pose
    wrist_right_pose = msg


if __name__ == '__main__':
    pub_hand_pose = rospy.Publisher('/hand_pose', Pose, queue_size=10)
    sub_l_wrist = rospy.Subscriber('/wrist_left', Pose, cb_l_wrist)
    sub_r_wrist = rospy.Subscriber('/wrist_right', Pose, cb_r_wrist)
    rospy.init_node('wrist_to_robot')
    rate = rospy.Rate(10.0)
    print "wrist_to_robot node started"

    mirror_state = None
    motion_state = None
    init_flag = False
    while not rospy.is_shutdown():
        if not mirror_state == 'y':
            print "Move to mirror pose (touch to robot tool). Ready: y"
            mirror_state = raw_input()
        else:
            if not init_flag:
                left_htm_init = DHmatrices.pose_to_htm(wrist_left_pose)
                right_htm_init = DHmatrices.pose_to_htm(wrist_right_pose)
                init_flag = True
            if not motion_state == 'y':
                print "Start motion? Start: y"
                motion_state = raw_input()
            else:
                tf_left = np.matmul(np.linalg.inv(left_htm_init), DHmatrices.pose_to_htm(wrist_left_pose))
                tf_left_pose = DHmatrices.htm_to_pose(tf_left)
                # hand_pose = DHmatrices.htm_to_pose(np.matmul(ur5e_init_htm, tf_left))
                pub_hand_pose.publish(tf_left_pose)
                # print "tf_left:", tf_left_pose
            
        rate.sleep()

