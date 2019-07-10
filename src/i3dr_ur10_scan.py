#!/usr/bin/env python

from math import pi
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import *
from control_msgs.msg import *
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import actionlib
import rospy
import time
import roslib
roslib.load_manifest('ur_driver')

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
joints_fold = [0, -175, 165, -180, 90, 90]
SCAN_JOINTS = []
joints_home = []
client = None
scan_pose_index = 0

move_UR10 = False
loop_scan = False
UR10_MOVE_HOME_POSE = 0
UR10_MOVE_FOLD_POSE = 1
UR10_MOVE_NEXT_POSE = 2
UR10_MOVE_SCAN_CONTINUOUS = 3
move_pose = UR10_MOVE_HOME_POSE

pub_i3dr_scan_status = rospy.Publisher('i3dr_scan_status', String, queue_size=10)

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

def handle_i3dr_scan_next_pose(req):
    global move_UR10, move_pose, UR10_MOVE_NEXT_POSE
    ur_move_cancel()
    print("request to move to next pose in scan")
    print(scan_pose_index)
    move_pose = UR10_MOVE_NEXT_POSE
    move_UR10 = True
    return EmptyResponse()

def handle_i3dr_scan_home_pose(req):
    global move_UR10, move_pose, UR10_MOVE_HOME_POSE
    ur_move_cancel()
    print("request to move to home pose")
    move_pose = UR10_MOVE_HOME_POSE
    move_UR10 = True
    return EmptyResponse()

def handle_i3dr_scan_fold_pose(req):
    global move_UR10, move_pose, UR10_MOVE_FOLD_POSE
    ur_move_cancel()
    print("request to move to fold pose")
    move_pose = UR10_MOVE_FOLD_POSE
    move_UR10 = True
    return EmptyResponse()

def handle_i3dr_scan_continuous(req):
    global move_UR10, move_pose, UR10_MOVE_SCAN_CONTINUOUS
    ur_move_cancel()
    print("request to scan continuous")
    move_pose = UR10_MOVE_SCAN_CONTINUOUS
    move_UR10 = True
    return EmptyResponse()

def handle_i3dr_scan_cancel_pose(req):
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
        if (not is_equal_joints(joints_pos,UR_Joints,1)):
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[
                0]*6, time_from_start=rospy.Duration(0.0))]
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=UR_Joints, velocities=[0]*6, time_from_start=rospy.Duration(time_to_pose)))

            client.send_goal(g)
            client.wait_for_result()
            pub_i3dr_scan_status.publish("position reached")
        else:
            print("already in position")
            pub_i3dr_scan_status.publish("already in position")
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def ur_move_home():
    global joints_home, scan_pose_index, ur10_move_time
    print "moving to home position"
    pub_i3dr_scan_status.publish("moving to home position...")
    print joints_home
    scan_pose_index = 0
    ur_move(joints_deg_to_rad(joints_home),ur10_move_time*2)

def ur_move_fold():
    global joints_fold, scan_pose_index, ur10_move_time
    print "moving to fold position"
    pub_i3dr_scan_status.publish("moving to folded position...")
    print joints_fold
    scan_pose_index = 0
    ur_move(joints_deg_to_rad(joints_fold),ur10_move_time*2)

def ur_move_scan_next_pose():
    global scan_pose_index, SCAN_JOINTS, ur10_move_time
    print "moving to next scan position"
    pub_i3dr_scan_status.publish("moving to next scan position...")
    print scan_pose_index
    joints = SCAN_JOINTS[scan_pose_index]
    print joints
    time_to_pose = ur10_move_time
    ur_move(joints_deg_to_rad(joints),time_to_pose)

    #iterate to next pose ready for next request
    scan_pose_index = scan_pose_index + 1
    if (scan_pose_index >= len(SCAN_JOINTS)):
        print("scan complete")
        scan_pose_index = 0

def ur_move_scan_continuous():
    global SCAN_JOINTS, move_UR10, loop_scan, ur10_pose_pause, ur10_move_time
    time_to_pose = ur10_move_time
    loop_scan = True
    pub_i3dr_scan_status.publish("starting continuous scan...")
    while loop_scan:
        for c,j in enumerate(SCAN_JOINTS):
            print j
            pub_i3dr_scan_status.publish("moving to next scan position...")
            ur_move(joints_deg_to_rad(j),time_to_pose)
            time.sleep(ur10_pose_pause)
            if not loop_scan:
                break
        if loop_scan:
            #loop_scan = False
            pub_i3dr_scan_status.publish("continuous scan complete")
            pub_i3dr_scan_status.publish("repeating continuous scan...")
            print("scan complete")
        else:
            pub_i3dr_scan_status.publish("continuous scan cancelled")
            print("scan cancelled")
        

def ur_move_cancel():
    global move_UR10, loop_scan
    print("canceling ur10 movement")
    pub_i3dr_scan_status.publish("cancelling movement...")
    move_UR10 = False
    loop_scan = False
    client.cancel_goal()
    pub_i3dr_scan_status.publish("movement cancelled.")

def generate_scan_joints_v(scan_joints,init_joints,target_joints,v_scan_angle):
    target_joints[5] = init_joints[5]
    scan_joints.append(target_joints[:])
    
    for i in range(0,2):
        target_joints[5] = target_joints[5]+v_scan_angle
        scan_joints.append(target_joints[:])
    for i in range(0,5):
        target_joints[5] = target_joints[5]-v_scan_angle
        scan_joints.append(target_joints[:])
    for i in range(0,3):
        target_joints[5] = target_joints[5]+v_scan_angle
        scan_joints.append(target_joints[:])
    target_joints[5] = init_joints[5]
    return(scan_joints,target_joints)
 
def generate_scan_joints(init_joints,h_scan_angle,v_scan_angle):
    scan_joints = []
    target_joints = init_joints

    scan_joints, target_joints = generate_scan_joints_v(scan_joints,init_joints,target_joints,v_scan_angle)
    
    for i in range(0,3):
        target_joints[0] = target_joints[0]-h_scan_angle
        scan_joints, t = generate_scan_joints_v(scan_joints,init_joints,target_joints,v_scan_angle)
    for i in range(0,6):
        target_joints[0] = target_joints[0]+h_scan_angle
        scan_joints, t = generate_scan_joints_v(scan_joints,init_joints,target_joints,v_scan_angle)
    for i in range(0,3):
        target_joints[0] = target_joints[0]-h_scan_angle
        scan_joints, t = generate_scan_joints_v(scan_joints,init_joints,target_joints,v_scan_angle)
    
    return(scan_joints)

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
            ur_move_scan_next_pose()
        elif (move_pose == UR10_MOVE_SCAN_CONTINUOUS):
            ur_move_scan_continuous()

def main():
    global client, SCAN_JOINTS, joints_home, ur10_move_time, ur10_pose_pause
    try:
        rospy.init_node("i3dr_ur10_sample_scan", anonymous=True, disable_signals=True)
        srv_i3dr_scan_next_pose = rospy.Service('i3dr_scan_next_pose', Empty, handle_i3dr_scan_next_pose)
        srv_i3dr_scan_home_pose = rospy.Service('i3dr_scan_home_pose', Empty, handle_i3dr_scan_home_pose)
        srv_i3dr_scan_fold_pose = rospy.Service('i3dr_scan_fold_pose', Empty, handle_i3dr_scan_fold_pose)
        srv_i3dr_scan_continuous = rospy.Service('i3dr_scan_continuous', Empty, handle_i3dr_scan_continuous)
        srv_i3dr_scan_cancel_pose = rospy.Service('i3dr_scan_cancel_pose', Empty, handle_i3dr_scan_cancel_pose)

        scan_type = rospy.get_param('~scan_type', 'room')
        ur10_move_time = rospy.get_param('~ur10_move_time', 10)
        ur10_pose_pause = rospy.get_param('~ur10_pose_pause', 0)

        if (scan_type == 'room'):
            joints_home = [0, -140, 90, -40, 0, 180]
            SCAN_JOINTS = generate_scan_joints(joints_home,30,20)
            #joints_home_rot = [180, -140, 90, -40, 0, 180]
            #SCAN_JOINTS_rot = generate_scan_joints(joints_home_rot,30,20)
            #SCAN_JOINTS.extend(SCAN_JOINTS_rot)
        elif (scan_type == 'sample'):
            joints_home = [0, -130, 90, -230, -90, -45]
            SCAN_JOINTS.append([-25,-100,100,-180,-85,-45])
            SCAN_JOINTS.append([0,-95,100,-190,-75,-45])

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
        rate = rospy.Rate(100)
        pub_i3dr_scan_status.publish("ready")
        while not rospy.is_shutdown():
            control_UR10()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__':
    main()
