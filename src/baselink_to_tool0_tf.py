#! /usr/bin/env python

"""
This is a node to publish tool0 pose with respect to base link
"""

import rospy

import math
import tf2_ros
from geometry_msgs.msg import Pose

tool0_actual_pose = Pose()


if __name__ == '__main__':
    pub_tool = rospy.Publisher('/base_to_tool', Pose, queue_size=10)
    rospy.init_node('base_to_tool_tf_node')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    print "Publishing tool actual pose (From robot, not calculated)"


    while not rospy.is_shutdown():
        try:
            trans_tool0 = tfBuffer.lookup_transform('base_link', 'tool0', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # print "Translation:", trans.transform.translation
        # print "Rotation:", trans.transform.rotation
        tool0_actual_pose.position = trans_tool0.transform.translation
        tool0_actual_pose.orientation = trans_tool0.transform.rotation
        pub_tool.publish(tool0_actual_pose)

        rate.sleep()

