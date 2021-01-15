#! /usr/bin/env python

import sys
import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown

joint_state_topic = ['joint_states:=/j2s7s300/joint_states']
roscpp_initialize(joint_state_topic)
rospy.init_node('kinova_record_poses', anonymous=False)

group = MoveGroupCommander('arm')
pose_target_arrs = []

while not rospy.is_shutdown():
    input_val = raw_input("a to add current pose to list. q to quit: ")
    print(input_val)
    if (input_val == 'a'):
        pose = group.get_current_pose()
        print(pose)

        pose_target_arr = [
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w
        ]
        pose_target_arrs.append(pose_target_arr)
    elif (input_val == 'q'):
        break

for i, pose_target_arr in enumerate(pose_target_arrs):
    print("pose_target_arr_"+str(i)+" = "+str(pose_target_arr))
pose_target_str = "pose_target_arrs = ["
for i, pose_target_arr in enumerate(pose_target_arrs):
    pose_target_str = pose_target_str + "pose_target_arr_" + str(i)
    if (i < len(pose_target_arrs)-1):
        pose_target_str = pose_target_str + ","
pose_target_str = pose_target_str + "]"
print(pose_target_str)

roscpp_shutdown()
