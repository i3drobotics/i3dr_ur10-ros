#!/usr/bin/env python

from math import pi
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import *
from control_msgs.msg import *
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import actionlib
import rospy
import time
import roslib
roslib.load_manifest('ur_driver')

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
joints_home = [0, -140, 90, -40, 0, 180]
joints_fold = [0, -175, 165, -180, 90, 90]
ROOM_SCAN_JOINTS = []
client = None
room_scan_pose_index = 0

move_UR10 = False
UR10_MOVE_HOME_POSE = 0
UR10_MOVE_FOLD_POSE = 1
UR10_MOVE_NEXT_POSE = 2
UR10_MOVE_SCAN_CONTINUOUS = 3
move_pose = UR10_MOVE_HOME_POSE

def deg_to_rad(deg):
    rad = deg * (3.14/180)
    return(rad)
    
def rad_to_deg(rad):
    deg = rad * (180/3.14)
    return(deg)

def joints_deg_to_rad(joints_deg):
    joints_rad = []
    for j in joints_deg:
        joints_rad.append(deg_to_rad(j))
    return(joints_rad)

def joints_rad_to_deg(joints_rad):
    joints_deg = []
    for j in joints_rad:
        joints_deg.append(rad_to_deg(j))
    return(joints_deg)

'''
def rtabmap_reset_odom():
    rospy.wait_for_service('/deimos/reset_odom  ')
    try:
        srv_reset_odom = rospy.ServiceProxy('/deimos/reset_odom  ', Empty)
        srv_reset_odom()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def rtabmap_new_map():
    rospy.wait_for_service('/deimos/trigger_new_map ')
    try:
        srv_new_map = rospy.ServiceProxy('/deimos/trigger_new_map ', Empty)
        srv_new_map()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def rtabmap_reset():
    rospy.wait_for_service('/deimos/reset')
    try:
        srv_reset = rospy.ServiceProxy('/deimos/reset', Empty)
        srv_reset()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def rtabmap_pause():
    rospy.wait_for_service('/deimos/pause')
    try:
        srv_reset = rospy.ServiceProxy('/deimos/pause', Empty)
        srv_reset()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def rtabmap_resume():
    rospy.wait_for_service('/deimos/resume')
    try:
        srv_reset = rospy.ServiceProxy('/deimos/resume', Empty)
        srv_reset()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
'''

def handle_i3dr_scan_room_next_pose(req):
    global move_UR10, move_pose, UR10_MOVE_NEXT_POSE
    ur_move_cancel()
    print("request to move to next pose in scan")
    print(room_scan_pose_index)
    move_pose = UR10_MOVE_NEXT_POSE
    move_UR10 = True
    return EmptyResponse()

def handle_i3dr_scan_room_home_pose(req):
    global move_UR10, move_pose, UR10_MOVE_HOME_POSE
    ur_move_cancel()
    print("request to move to home pose")
    move_pose = UR10_MOVE_HOME_POSE
    move_UR10 = True
    return EmptyResponse()

def handle_i3dr_scan_room_fold_pose(req):
    global move_UR10, move_pose, UR10_MOVE_FOLD_POSE
    ur_move_cancel()
    print("request to move to fold pose")
    move_pose = UR10_MOVE_FOLD_POSE
    move_UR10 = True
    return EmptyResponse()

def handle_i3dr_scan_room_continuous(req):
    global move_UR10, move_pose, UR10_MOVE_SCAN_CONTINUOUS
    ur_move_cancel()
    print("request to scan continuous")
    move_pose = UR10_MOVE_SCAN_CONTINUOUS
    move_UR10 = True
    return EmptyResponse()

def handle_i3dr_scan_room_cancel_pose(req):
    global move_UR10
    print("request to cancel movement")
    ur_move_cancel()
    return EmptyResponse()

def rtabmap_init():
    print "resetting rtabmap"
    rtabmap_reset()

def ur_move_deg(UR_Joints,time_to_pose):
    print "moving to position"
    print UR_Joints
    UR_Joints = joints_deg_to_rad(UR_Joints)
    ur_move(UR_Joints,time_to_pose)

def ur_move_rad(UR_Joints,time_to_pose):
    print "moving to position"
    print UR_Joints
    ur_move(UR_Joints,time_to_pose)

def is_equal_joints(joint_a,joint_b,precision):
    joint_a_round = []
    joint_b_round = []
    for j_a in joint_a:
        a = round(j_a,precision)
        joint_a_round.append(a)
    for j_b in joint_b:
        b = round(j_b,precision)
        joint_b_round.append(b)
    print joint_a_round
    print joint_b_round
    if (joint_a_round == joint_b_round):
        return True
    else:
        return False

def ur_move(UR_Joints,time_to_pose):
    try:
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES

        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        if (not is_equal_joints(joints_pos,UR_Joints,2)):
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[
                0]*6, time_from_start=rospy.Duration(0.0))]
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=UR_Joints, velocities=[0]*6, time_from_start=rospy.Duration(time_to_pose)))

            client.send_goal(g)
            client.wait_for_result()
        else:
            print("already in position")
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def ur_move_home():
    global joints_home, room_scan_pose_index
    print "moving to home position"
    print joints_home
    room_scan_pose_index = 0
    ur_move(joints_deg_to_rad(joints_home),10)

def ur_move_fold():
    global joints_fold, room_scan_pose_index
    print "moving to fold position"
    print joints_fold
    room_scan_pose_index = 0
    ur_move(joints_deg_to_rad(joints_fold),10)

def ur_move_room_scan_next_pose():
    global room_scan_pose_index, ROOM_SCAN_JOINTS
    print "moving to next room scan position"
    joints = ROOM_SCAN_JOINTS[room_scan_pose_index]
    print joints
    time_to_pose = 5
    if (room_scan_pose_index == 0):
        time_to_pose = 10
    ur_move(joints_deg_to_rad(joints),time_to_pose)

    #iterate to next pose ready for next request
    room_scan_pose_index = room_scan_pose_index + 1
    if (room_scan_pose_index > len(ROOM_SCAN_JOINTS)):
        print("room scan complete")
        room_scan_pose_index = 0

def ur_move_room_scan_continuous():
    global ROOM_SCAN_JOINTS, move_UR10
    time_to_pose = 5
    while move_UR10:
        for c,j in enumerate(ROOM_SCAN_JOINTS):
            print j
            if (c == 0):
                time_to_pose = 10
            else: 
                time_to_pose = 5
            ur_move(joints_deg_to_rad(j),time_to_pose)
            if not move_UR10:
                break
    if move_UR10:
        move_UR10 = False
        print("room scan complete")
    else:
        print("room scan cancelled")
        

def ur_move_cancel():
    global move_UR10
    print("canceling ur10 movement")
    move_UR10 = False
    client.cancel_goal()

def generate_scan_room_joints_v(scan_room_joints,init_joints,target_joints,v_scan_angle):
    target_joints[5] = init_joints[5]
    scan_room_joints.append(target_joints[:])
    
    for i in range(0,2):
        target_joints[5] = target_joints[5]+v_scan_angle
        scan_room_joints.append(target_joints[:])
    for i in range(0,3):
        target_joints[5] = target_joints[5]-v_scan_angle
        scan_room_joints.append(target_joints[:])
    for i in range(0,1):
        target_joints[5] = target_joints[5]+v_scan_angle
        scan_room_joints.append(target_joints[:])
    target_joints[5] = init_joints[5]
    return(scan_room_joints,target_joints)

def generate_scan_room_joints(init_joints,h_scan_angle,v_scan_angle):
    scan_room_joints = []
    target_joints = init_joints

    scan_room_joints, target_joints = generate_scan_room_joints_v(scan_room_joints,init_joints,target_joints,v_scan_angle)
    
    for i in range(0,2):
        target_joints[0] = target_joints[0]-h_scan_angle
        scan_room_joints, t = generate_scan_room_joints_v(scan_room_joints,init_joints,target_joints,v_scan_angle)
    for i in range(0,4):
        target_joints[0] = target_joints[0]+h_scan_angle
        scan_room_joints, t = generate_scan_room_joints_v(scan_room_joints,init_joints,target_joints,v_scan_angle)
    for i in range(0,2):
        target_joints[0] = target_joints[0]-h_scan_angle
        scan_room_joints, t = generate_scan_room_joints_v(scan_room_joints,init_joints,target_joints,v_scan_angle)
    
    return(scan_room_joints)

def control_UR10():
    global move_UR10, UR10_MOVE_HOME_POSE, UR10_MOVE_FOLD_POSE, UR10_MOVE_NEXT_POSE, UR10_MOVE_SCAN_CONTINUOUS
    if (move_UR10):
        if (move_pose == UR10_MOVE_HOME_POSE):
            move_UR10 = False
            ur_move_home()
        elif (move_pose == UR10_MOVE_FOLD_POSE):
            move_UR10 = False
            ur_move_fold()
        elif (move_pose == UR10_MOVE_NEXT_POSE):
            move_UR10 = False
            ur_move_room_scan_next_pose()
        elif (move_pose == UR10_MOVE_SCAN_CONTINUOUS):
            ur_move_room_scan_continuous()

def main():
    global client, ROOM_SCAN_JOINTS
    try:
        rospy.init_node("i3dr_ur10_room_scan", anonymous=True, disable_signals=True)
        srv_i3dr_scan_room_next_pose = rospy.Service('i3dr_scan_room_next_pose', Empty, handle_i3dr_scan_room_next_pose)
        srv_i3dr_scan_room_home_pose = rospy.Service('i3dr_scan_room_home_pose', Empty, handle_i3dr_scan_room_home_pose)
        srv_i3dr_scan_room_fold_pose = rospy.Service('i3dr_scan_room_fold_pose', Empty, handle_i3dr_scan_room_fold_pose)
        srv_i3dr_scan_room_continuous = rospy.Service('i3dr_scan_room_continuous', Empty, handle_i3dr_scan_room_continuous)
        srv_i3dr_scan_room_cancel_pose = rospy.Service('i3dr_scan_room_cancel_pose', Empty, handle_i3dr_scan_room_cancel_pose)

        ROOM_SCAN_JOINTS = generate_scan_room_joints(joints_home,20,20)
        print(ROOM_SCAN_JOINTS)

        client = actionlib.SimpleActionClient(
            'follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(
                index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print "This program makes the robot move to scan the environment"
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):
            rate = rospy.Rate(100)
            while not rospy.is_shutdown():
                control_UR10()
                rate.sleep()
        else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__':
    main()
