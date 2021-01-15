#! /usr/bin/env python

import sys
import rospy
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown

joint_state_topic = ['joint_states:=/j2s7s300/joint_states']
roscpp_initialize(joint_state_topic)
rospy.init_node('kinova_move', anonymous=False)

group = MoveGroupCommander('arm')

'''
pose_target_arr_1 = [0, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5]
pose_target_arr_2 = [0.3, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5]
pose_target_arr_3 = [-0.3, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5]
pose_target_arr_4 = [0.3, 0.7, 0.5, -0.5, 0.5, 0.5, 0.5]
pose_target_arr_5 = [0, 0.5, 0.3, -0.5, 0.5, 0.5, 0.5]
pose_target_arrs = [
    pose_target_arr_1, pose_target_arr_2, pose_target_arr_3, pose_target_arr_4,
    pose_target_arr_1, pose_target_arr_5,
    pose_target_arr_1
]
'''

pose_target_arr_0 = [-0.16319405389239938, 0.21536368834049663, 0.5412445673626896, -0.43965123159822456, 0.5453534139953018, 0.4757910264580598, 0.5319016333306558]
pose_target_arr_1 = [-0.4804366005739147, 0.24835380484911984, 0.5562890762240151, -0.495874414423256, 0.49055181324342323, 0.5304360357001974, 0.4817728673084875]
pose_target_arr_2 = [-0.5631582277935051, 0.17375187832323016, 0.55753959412313, -0.6081722340598132, 0.32717661215392096, 0.6436439366348544, 0.32985524251005305]
pose_target_arr_3 = [-0.3650909918562459, 0.2001852302241025, 0.5496401723960802, -0.5134177304706955, 0.46982917952431585, 0.5489787947722521, 0.46290934209220047]
pose_target_arr_4 = [-0.1851821388000565, 0.2246404324359434, 0.5421967668035881, -0.5112471449537022, 0.47164328932132826, 0.5508914593087679, 0.4611914618415025]
pose_target_arr_5 = [0.14733121110120911, 0.3063612824721158, 0.5281530873839351, -0.38453132639280974, 0.5898856509142734, 0.4255956781510819, 0.5683651085412886]
pose_target_arr_6 = [0.42974107081373203, 0.17466124013212, 0.5051167868512337, -0.5823404257500746, 0.37122180791706927, 0.6233316449788355, 0.3667855752779594]
pose_target_arr_7 = [0.5240302011864642, 0.14122849694333978, 0.4939222201580085, -0.3939934636142883, 0.5860227605329958, 0.4328724317165172, 0.5603284149731589]
pose_target_arr_8 = [0.46499548466273377, -0.059201721898677076, 0.6473108953552635, -0.2297704355432089, 0.6770643050563439, 0.2660335704490048, 0.6465412694971672]
pose_target_arr_9 = [0.4606104565702859, -0.20368798627012494, 0.6355318815864357, 0.019009865230522532, 0.7336206099593224, 0.007638431128291888, 0.6792503809617831]
pose_target_arr_10 = [0.39741454674585075, -0.3046908612841809, 0.6049153668918237, 0.2664445623803424, 0.7111600885433395, -0.20885619604133057, 0.6161474766775588]
pose_target_arrs = [pose_target_arr_0,pose_target_arr_1,pose_target_arr_2,pose_target_arr_3,pose_target_arr_4,pose_target_arr_5,pose_target_arr_6,pose_target_arr_7,pose_target_arr_8,pose_target_arr_9,pose_target_arr_10]


while not rospy.is_shutdown():
    for pose_target_arr in pose_target_arrs:
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = pose_target_arr[0]
        pose_target.position.y = pose_target_arr[1]
        pose_target.position.z = pose_target_arr[2]
        pose_target.orientation.x = pose_target_arr[3]
        pose_target.orientation.y = pose_target_arr[4]
        pose_target.orientation.z = pose_target_arr[5]
        pose_target.orientation.w = pose_target_arr[6]

        group.set_pose_target(pose_target)
        group.set_max_velocity_scaling_factor(0.5)

        plan2 = group.plan()
        group.go(wait=True)

        rospy.sleep(2)

roscpp_shutdown()
