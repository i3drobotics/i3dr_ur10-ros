#! /usr/bin/env python
import math
import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown


def rotate_joint(joint_in, angle, joint_index):
    joint_out = joint_in[:]
    joint_out[joint_index] = joint_out[joint_index]+angle
    return joint_out


JOINT_INDEX_BASE = 0
JOINT_INDEX_SHOULDER = 1
JOINT_INDEX_LOWER_ARM = 2
JOINT_INDEX_ELBOW = 3
JOINT_INDEX_UPPER_ARM = 4
JOINT_INDEX_WRIST_YAW = 5
JOINT_INDEX_WRIST_ROLL = 6

joint_state_topic = ['joint_states:=/j2s7s300/joint_states']
roscpp_initialize(joint_state_topic)
rospy.init_node('kinova_move', anonymous=False)

group = MoveGroupCommander('arm')

joint_target_init = [0, 2.8, 0.2, 1.4, 4.5, 1.8, 0.2]

joint_target_1 = rotate_joint(joint_target_init, 0.2, JOINT_INDEX_UPPER_ARM)
joint_target_2 = rotate_joint(joint_target_init, -0.2, JOINT_INDEX_UPPER_ARM)
joint_target_3 = rotate_joint(joint_target_init, math.pi/4, JOINT_INDEX_BASE)
joint_target_4 = rotate_joint(joint_target_3, 0.2, JOINT_INDEX_UPPER_ARM)
joint_target_5 = rotate_joint(joint_target_3, -0.2, JOINT_INDEX_UPPER_ARM)
joint_targets = [
    joint_target_init,
    joint_target_1,
    joint_target_2,
    joint_target_3,
    joint_target_4,
    joint_target_5,
    joint_target_init
]

joint_targets = []
for i in range(0, 361, 45):
    for j in range(-20, 21, 20):
        i_rads = i * math.pi/180
        j_rads = j * math.pi/180
        joint_target = rotate_joint(
            joint_target_init, i_rads, JOINT_INDEX_BASE)
        joint_target = rotate_joint(
            joint_target, j_rads, JOINT_INDEX_UPPER_ARM)
        joint_targets.append(joint_target)


for joint_target in joint_targets:

    group.set_joint_value_target(joint_target)
    group.set_max_velocity_scaling_factor(0.1)

    plan = group.plan()
    group.go(wait=True)

    group.stop()

    if rospy.is_shutdown():
        break

    rospy.sleep(1)

roscpp_shutdown()
