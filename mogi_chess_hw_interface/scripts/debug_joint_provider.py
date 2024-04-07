#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

def get_joints (msg):
    if (len(msg.name) == 0):
        rospy.loginfo("No joints were detected!")
    else:
        rospy.loginfo(msg.name)
        #rospy.loginfo(msg.position)
        for i, joint_name in enumerate(msg.name):
            if joint_name == "shoulder_pan_joint":
                pub_shoulder_pan.publish(msg.position[i])

rospy.init_node('debug_joint_provider')

sub_joints = rospy.Subscriber ('/joint_states', JointState, get_joints)
pub_shoulder_pan = rospy.Publisher('/debug_shoulder_pan_angle', Float64, queue_size=10)
#[INFO] [1712481101.475517]: ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#[INFO] [1712481101.477159]: ['rh_p12_rn_a', 'rh_r2', 'rh_l1', 'rh_l2', 'gripper']



rospy.spin()