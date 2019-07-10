#!/usr/bin/env python
#
# Copyright 2015, 2016 Thomas Timm Andersen (original version)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from math import pi
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import *
from control_msgs.msg import *
from std_srvs.srv import Empty
import actionlib
import rospy
import time
import roslib
roslib.load_manifest('ur_driver')

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
joints_home = [0, -0.872665, -1.5708, -2.26893, -1.5708, 0]
client = None

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

def rtabmap_init():
    print "resetting rtabmap"
    rtabmap_reset()

def ur_move(UR_Joints,time_to_pose):
    try:
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES

        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[
            0]*6, time_from_start=rospy.Duration(0.0))]
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=UR_Joints, velocities=[0]*6, time_from_start=rospy.Duration(time_to_pose)))

        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def ur_move_home():
    print "moving to home position"
    ur_move(joints_home,20)

def ur_move_scan():
    print "starting scan movements"
    ur_joints = joints_home
    d = 8
    for i in range(0, 3):
        for w in range(160, 90, -20):
            wrist_1_joint = -w * (3.14/180)
            ur_joints[3] = wrist_1_joint

            for b in range(0, 360, 40):
                shoulder_pan_joint = -b * (3.14/180)
                ur_joints[0] = shoulder_pan_joint

                print ur_joints
                ur_move(ur_joints,d)

            for b in range(360, 0, -40):
                shoulder_pan_joint = -b * (3.14/180)
                ur_joints[0] = shoulder_pan_joint

                print ur_joints
                ur_move(ur_joints,d)

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
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
            rtabmap_pause()
            ur_move_home()
            rtabmap_init()
            rtabmap_resume()
            ur_move_scan()
        else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__':
    main()
