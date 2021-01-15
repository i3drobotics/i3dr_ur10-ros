#! /usr/bin/env python

import sys
import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown

joint_state_topic = ['joint_states:=/j2s7s300/joint_states']
roscpp_initialize(joint_state_topic)
rospy.init_node('kinova_get_pose', anonymous=False)

group = MoveGroupCommander('arm')

pose = group.get_current_pose()
print(pose)

rospy.sleep(1)

roscpp_shutdown()
