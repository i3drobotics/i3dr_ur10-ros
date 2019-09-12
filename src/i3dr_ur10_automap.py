#!/usr/bin/env python

from math import pi
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from rtabmap_ros.srv import GetMap, GetMapRequest, GetMapResponse
import actionlib
import rospy
import time
import roslib
import rosparam
import rosservice
roslib.load_manifest('ur_driver')

class UR10Automap:
    def __init__(self):
        self.setupNode()

    def setupNode(self):
        self.client = None
        self.SCAN_JOINTS = []
        self.joints_home = []
        self.rtabmap_is_paused = False
        self.rtabmap_pause_en = False
        self.rtabmap_send_data = True
        self.cancel_loop = False
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joints_fold = [0, -175, 165, -180, 90, 90]
        self.scan_pose_index = 0

        self.rtabmap_cloud = None

        self.move_UR10 = False
        self.UR10_MOVE_HOME_POSE = 0
        self.UR10_MOVE_FOLD_POSE = 1
        self.UR10_MOVE_NEXT_POSE = 2
        self.UR10_MOVE_SCAN_CONTINUOUS = 3
        self.move_pose = self.UR10_MOVE_HOME_POSE

        self.queue_size = 10

        self.rtabmap_inputs_received = [False, False, False, False]

        rospy.init_node("i3dr_ur10_automap", anonymous=True,
                        disable_signals=True)
        rospy.Service('i3dr_scan_next_pose', Empty,
                      self.handle_i3dr_scan_next_pose)
        rospy.Service('i3dr_scan_home_pose', Empty,
                      self.handle_i3dr_scan_home_pose)
        rospy.Service('i3dr_scan_fold_pose', Empty,
                      self.handle_i3dr_scan_fold_pose)
        rospy.Service('i3dr_scan_continuous', Empty,
                      self.handle_i3dr_scan_continuous)
        rospy.Service('i3dr_scan_cancel_pose', Empty,
                      self.handle_i3dr_scan_cancel_pose)

        self.rtabmap_enable = rospy.get_param('~rtabmap_en', True)
        self.loop_scan = rospy.get_param('~loop_scan', False)
        # Automatically send map at end of scan
        self.auto_send_map = rospy.get_param('~auto_send_map', False)
        self.ur10_move_time = rospy.get_param('~ur10_move_time', 10)
        self.ur10_pose_pause = rospy.get_param('~ur10_pose_pause', 0)
        self.rtabmap_namespace = rospy.get_param(
            '~rtabmap_namespace', "rtabmap")
        #TODO remove this when routine loading from yaml is setup
        self.routine = rospy.get_param('~routine',"bottom")

        self.rgb_topic = "left/image_rect"
        self.depth_topic = "depth"
        self.camera_info_topic = "left/camera_info"
        self.depth_camera_info_topic = "left/depth_camera_info"
        self.rtabmap_rgb_topic = "left/image_rect_rtabmap"
        self.rtabmap_depth_topic = "depth_rtabmap"
        self.rtabmap_camera_info_topic = "left/camera_info_rtabmap"
        self.rtabmap_depth_camera_info_topic = "left/depth_camera_info_rtabmap"

        # Subscribe to data needed for rtabmap
        self.rgb_topic_sub = rospy.Subscriber(
            self.rgb_topic, Image, self.callback_rgb_topic)
        self.depth_topic_sub = rospy.Subscriber(
            self.depth_topic, Image, self.callback_depth_topic)
        self.camera_info_topic_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.callback_camera_info_topic)
        self.depth_camera_info_topic_sub = rospy.Subscriber(
            self.depth_camera_info_topic, Image, self.callback_depth_camera_info_topic)
        # Publishers for sending data to rtabmap
        # self.rtabmap_rgb_topic_pub = rospy.Publisher(self.rtabmap_rgb_topic,Image,queue_size=self.queue_size)
        # self.rtabmap_depth_topic_pub = rospy.Publisher(self.rtabmap_depth_topic,Image,queue_size=self.queue_size)
        # self.rtabmap_camera_info_topic_pub = rospy.Publisher(self.rtabmap_camera_info_topic,CameraInfo,queue_size=self.queue_size)
        # self.rtabmap_depth_camera_info_topic_pub = rospy.Publisher(self.rtabmap_depth_camera_info_topic,CameraInfo,queue_size=self.queue_size)

        self.pub_i3dr_scan_status = rospy.Publisher(
            'i3dr_scan_status', String, queue_size=self.queue_size)

        if (self.routine == "bottom_rov_mount"):
          # setup joints for map
          self.joints_home = [0, -70, -80, -120, 90, -90]

          self.SCAN_JOINTS.append(self.joints_home)
          wrist_angle_home = self.joints_home[5]
          move_angle = 5
          for j in range(wrist_angle_home, wrist_angle_home+90, move_angle):
              self.SCAN_JOINTS.append([0, -70, -80, -120, 90, j])
          for j in range(wrist_angle_home+90, wrist_angle_home-90, -move_angle):
              self.SCAN_JOINTS.append([0, -70, -80, -120, 90, j])
          for j in range(wrist_angle_home-90, wrist_angle_home, move_angle):
              self.SCAN_JOINTS.append([0, -70, -80, -120, 90, j])
        elif (self.routine == "top"):
          self.joints_home = [0, 0, 0, 0, 0, 0]
          self.SCAN_JOINTS.append(self.joints_home)
          #TODO add joints here for top routine
          self.SCAN_JOINTS.append([0, 0, 0, 0, 0, 0])
        elif (self.routine == "file"):
          #TODO load routine from yaml instead of hard coded
          self.routine_yaml_file = rospy.get_param('~routine_info_yaml')
          self.routine_yaml_data = rosparam.load_file(self.routine_yaml_file)
          rospy.set_param("joint_params",self.routine_yaml_data)
          self.routine_info = rospy.get_param('joint_params')[0][0]
          print(self.routine_info)
          self.routine_joints = self.routine_info["joints_routine"]
          print(self.routine_joints)
          self.joints_home = self.routine_info["joints_home"]

          for r in self.routine_joints:
            self.SCAN_JOINTS.append(r)

        self.client = actionlib.SimpleActionClient(
            'follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(
                index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(self.JOINT_NAMES):
                self.JOINT_NAMES[i] = prefix + name
        print "This program makes the robot move to scan the environment"
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        if (self.rtabmap_enable):
            print "Setting up rtabmap"
            self.rtabmap_new_map()
            self.rtabmap_reset()
            self.rtabmap_reset_odom()
        self.pub_i3dr_scan_status.publish("ready")
        print "Ready"

    def spin(self, rate_hz):
        rate = rospy.Rate(rate_hz)
        try:
            while not rospy.is_shutdown():
                self.control_UR10()
                rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    # IMAGE SUBSCRIBER CALLBACKS
    # ------------------------------------- #
    def callback_rgb_topic(self, data):
        self.rtabmap_inputs_received[0] = True

    def callback_depth_topic(self, data):
        self.rtabmap_inputs_received[1] = True

    def callback_camera_info_topic(self, data):
        self.rtabmap_inputs_received[2] = True

    def callback_depth_camera_info_topic(self, data):
        self.rtabmap_inputs_received[3] = True

    ## RTABMAP ##
    # ------------------------------------- #

    def rtabmap_service_exists(self):
        service_list = rosservice.get_service_list()
        reset_odom_service = '/'+self.rtabmap_namespace+'/reset_odom'
        if (reset_odom_service in service_list):
            return True
        else:
            return False

    def rtabmap_send_map(self):
        try:
            rospy.wait_for_service('i3dr_scan_send_map', timeout=2)
            srv_send_map = rospy.ServiceProxy('i3dr_scan_send_map', Empty)
            srv_send_map()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)
        except:
            rospy.logerr("Service call failed: Unknown error")

    def rtabmap_get_map(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/get_map_data', timeout=2)
            srv_get_map = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/get_map_data', GetMap)
            req = GetMapRequest(True, True, False)
            # request map and wait for response
            resp = srv_get_map(req)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)
        except:
            rospy.logerr("Service call failed: Unknown error")

    def rtabmap_reset_odom(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/reset_odom', timeout=2)
            srv_reset_odom = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/reset_odom', Empty)
            srv_reset_odom()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_new_map(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/trigger_new_map', timeout=2)
            srv_new_map = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/trigger_new_map', Empty)
            srv_new_map()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_reset(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/reset', timeout=2)
            srv_reset = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/reset', Empty)
            srv_reset()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_pause(self):
        print "Pausing rtabmap with service: /"+self.rtabmap_namespace+"/pause"
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/pause', timeout=2)
            srv_pause = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/pause', Empty)
            srv_pause()
            self.rtabmap_is_paused = True
            print "Rtabmap service complete: /"+self.rtabmap_namespace+"/pause"
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_pause_odom(self):
        print "Pausing rtabmap odometry with service: /" + \
            self.rtabmap_namespace+"/pause_odom"
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/pause_odom', timeout=2)
            srv_pause_odom = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/pause_odom', Empty)
            srv_pause_odom()
            self.rtabmap_is_paused = True
            print "Rtabmap service complete: /"+self.rtabmap_namespace+"/pause_odom"
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_resume(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/resume', timeout=2)
            srv_resume = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/resume', Empty)
            srv_resume()
            self.rtabmap_is_paused = False
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_resume_odom(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/resume_odom', timeout=2)
            srv_reset = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/resume_odom', Empty)
            srv_reset()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    # ------------------------------------- #

    # HANDLE SERVICE CALLS
    # ------------------------------------- #

    def handle_i3dr_scan_next_pose(self, req):
        self.ur_move_cancel()
        print("request to move to next pose in scan")
        print(self.scan_pose_index)
        self.move_pose = self.UR10_MOVE_NEXT_POSE
        self.move_UR10 = True
        return EmptyResponse()

    def handle_i3dr_scan_home_pose(self, req):
        self.ur_move_cancel()
        print("request to move to home pose")
        self.move_pose = self.UR10_MOVE_HOME_POSE
        self.move_UR10 = True
        return EmptyResponse()

    def handle_i3dr_scan_fold_pose(self, req):
        self.ur_move_cancel()
        print("request to move to fold pose")
        self.move_pose = self.UR10_MOVE_FOLD_POSE
        self.move_UR10 = True
        return EmptyResponse()

    def handle_i3dr_scan_continuous(self, req):
        self.ur_move_cancel()
        print("request to scan continuous")
        self.move_pose = self.UR10_MOVE_SCAN_CONTINUOUS
        self.move_UR10 = True
        return EmptyResponse()

    def handle_i3dr_scan_cancel_pose(self, req):
        print("request to cancel movement")
        self.ur_move_cancel()
        return EmptyResponse()

    def handle_i3dr_reset_map(self, req):
        print("request to reset map")
        self.rtabmap_new_map()
        self.rtabmap_reset()
        self.rtabmap_reset_odom()
        return EmptyResponse()

    def handle_i3dr_pause_map(self, req):
        print("request to pause map")
        self.rtabmap_pause_en = True

    def handle_i3dr_resume_map(self, req):
        print("request to resume map")
        self.rtabmap_pause_en = False

    # ------------------------------------- #

    def deg_to_rad(self, deg):
        rad = deg * (3.14/180)
        return(rad)

    def rad_to_deg(self, rad):
        deg = rad * (180/3.14)
        return(deg)

    def joints_deg_to_rad(self, joints_deg):
        joints_rad = []
        for j in joints_deg:
            joints_rad.append(self.deg_to_rad(j))
        return(joints_rad)

    def joints_rad_to_deg(self, joints_rad):
        joints_deg = []
        for j in joints_rad:
            joints_deg.append(self.rad_to_deg(j))
        return(joints_deg)

    def ur_move_deg(self, UR_Joints, time_to_pose):
        print "moving to position"
        print UR_Joints
        UR_Joints = self.joints_deg_to_rad(UR_Joints)
        self.ur_move(UR_Joints, time_to_pose)

    def ur_move_rad(self, UR_Joints, time_to_pose):
        print "moving to position"
        print UR_Joints
        self.ur_move(UR_Joints, time_to_pose)

    def is_equal_joints(self, joint_a, joint_b, precision):
        joint_a_round = []
        joint_b_round = []
        for j_a in joint_a:
            a = round(j_a, precision)
            joint_a_round.append(a)
        for j_b in joint_b:
            b = round(j_b, precision)
            joint_b_round.append(b)
        # print joint_a_round
        # print joint_b_round
        if (joint_a_round == joint_b_round):
            return True
        else:
            return False

    def ur_move(self, UR_Joints, time_to_pose):
        try:
            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = self.JOINT_NAMES

            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            if (not self.is_equal_joints(joints_pos, UR_Joints, 1)):
                g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[
                    0]*6, time_from_start=rospy.Duration(0.0))]
                g.trajectory.points.append(
                    JointTrajectoryPoint(positions=UR_Joints, velocities=[0]*6, time_from_start=rospy.Duration(time_to_pose)))

                self.client.send_goal(g)
                self.client.wait_for_result()
                self.pub_i3dr_scan_status.publish("position reached")
            else:
                print("already in position")
                self.pub_i3dr_scan_status.publish("already in position")
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def ur_move_home(self):
        print "moving to home position"
        self.pub_i3dr_scan_status.publish("moving to home position...")
        print self.joints_home
        self.scan_pose_index = 0

        if (self.rtabmap_enable):
            if (not self.rtabmap_pause_en):
                # Pause sending data to RTABMAP
                self.rtabmap_pause()
                self.rtabmap_pause_odom()
                # self.rtabmap_send_data = False

        self.ur_move(self.joints_deg_to_rad(
            self.joints_home), self.ur10_move_time*2)

        if (self.rtabmap_enable):
            if (not self.rtabmap_pause_en):
                rospy.sleep(0.5)  # small pause to make sure robot has stopped
                received = self.wait_for_rtabmap_inputs(self.ur10_pose_pause)
                if (received):
                    self.rtabmap_resume()
                    self.rtabmap_resume_odom()
                    self.wait_for_rtabmap_inputs(self.ur10_pose_pause)

    def ur_move_fold(self):
        print "moving to fold position"
        self.pub_i3dr_scan_status.publish("moving to folded position...")
        print self.joints_fold
        self.scan_pose_index = 0

        if (self.rtabmap_enable):
            if (not self.rtabmap_pause_en):
                # Pause sending data to RTABMAP
                self.rtabmap_pause()
                self.rtabmap_pause_odom()
                # self.rtabmap_send_data = False

        self.ur_move(self.joints_deg_to_rad(
            self.joints_fold), self.ur10_move_time*2)

    def ur_move_scan_next_pose(self):
        print "moving to next scan position"
        self.pub_i3dr_scan_status.publish("moving to next scan position...")
        print self.scan_pose_index
        joints = self.SCAN_JOINTS[self.scan_pose_index]
        print joints
        time_to_pose = self.ur10_move_time

        if (self.rtabmap_enable):
            if (not self.rtabmap_pause_en):
                # Pause sending data to RTABMAP
                self.rtabmap_pause()
                self.rtabmap_pause_odom()
                # self.rtabmap_send_data = False

        self.ur_move(self.joints_deg_to_rad(joints), time_to_pose)

        # iterate to next pose ready for next request
        self.scan_pose_index = self.scan_pose_index + 1
        if (self.scan_pose_index >= len(self.SCAN_JOINTS)):
            print("scan complete")
            self.scan_pose_index = 0

        if (self.rtabmap_enable):
            if (not self.rtabmap_pause_en):
                rospy.sleep(0.5)  # small pause to make sure robot has stopped
                received = self.wait_for_rtabmap_inputs(self.ur10_pose_pause)
                if (received):
                    self.rtabmap_resume()
                    self.rtabmap_resume_odom()
                    self.wait_for_rtabmap_inputs(self.ur10_pose_pause)

    def ur_move_scan_continuous(self):
        time_to_pose = self.ur10_move_time
        self.pub_i3dr_scan_status.publish("starting scan...")
        local_loop_scan = True
        self.cancel_loop = False
        while local_loop_scan:
            for j in self.SCAN_JOINTS:
                print j
                self.pub_i3dr_scan_status.publish(
                    "moving to next scan position...")

                if (self.rtabmap_enable):
                    if (not self.rtabmap_pause_en):
                        # Pause sending data to RTABMAP
                        self.rtabmap_pause()
                        self.rtabmap_pause_odom()
                        # self.rtabmap_send_data = False

                self.ur_move(self.joints_deg_to_rad(j), time_to_pose)

                if (self.rtabmap_enable):
                    if (not self.rtabmap_pause_en):
                        # small pause to make sure robot has stopped
                        rospy.sleep(0.5)
                        received = self.wait_for_rtabmap_inputs(
                            self.ur10_pose_pause)
                        if (received):
                            self.rtabmap_resume()
                            self.rtabmap_resume_odom()
                            self.wait_for_rtabmap_inputs(self.ur10_pose_pause)

                if self.cancel_loop:
                    break

            if self.cancel_loop:
                self.pub_i3dr_scan_status.publish("scan cancelled")
                print("scan cancelled")
                break

            local_loop_scan = self.loop_scan

        if not self.cancel_loop:
            self.pub_i3dr_scan_status.publish("scan complete")
            print("scan complete")
            self.move_UR10 = False
            if self.loop_scan:
                self.pub_i3dr_scan_status.publish(
                    "repeating continuous scan...")
            else:
                if self.auto_send_map:
                    self.rtabmap_send_map()

    def ur_move_cancel(self):
        print("canceling ur10 movement")
        self.pub_i3dr_scan_status.publish("cancelling movement...")
        self.move_UR10 = False
        self.cancel_loop = True
        self.client.cancel_goal()
        self.pub_i3dr_scan_status.publish("movement cancelled.")

    def generate_scan_joints_v(self, scan_joints, init_joints, target_joints, v_scan_angle):
        target_joints[5] = init_joints[5]
        scan_joints.append(target_joints[:])

        for i in range(0, 2):
            target_joints[5] = target_joints[5]+v_scan_angle
            scan_joints.append(target_joints[:])
        for i in range(0, 5):
            target_joints[5] = target_joints[5]-v_scan_angle
            scan_joints.append(target_joints[:])
        for i in range(0, 3):
            target_joints[5] = target_joints[5]+v_scan_angle
            scan_joints.append(target_joints[:])
        target_joints[5] = init_joints[5]
        return(scan_joints, target_joints)

    def generate_scan_joints(self, init_joints, h_scan_angle, v_scan_angle):
        scan_joints = []
        target_joints = init_joints

        scan_joints, target_joints = self.generate_scan_joints_v(
            scan_joints, init_joints, target_joints, v_scan_angle)

        for i in range(0, 3):
            target_joints[0] = target_joints[0]-h_scan_angle
            scan_joints, t = self.generate_scan_joints_v(
                scan_joints, init_joints, target_joints, v_scan_angle)
        for i in range(0, 6):
            target_joints[0] = target_joints[0]+h_scan_angle
            scan_joints, t = self.generate_scan_joints_v(
                scan_joints, init_joints, target_joints, v_scan_angle)
        for i in range(0, 3):
            target_joints[0] = target_joints[0]-h_scan_angle
            scan_joints, t = self.generate_scan_joints_v(
                scan_joints, init_joints, target_joints, v_scan_angle)

        return(scan_joints)

    def wait_for_rtabmap_inputs(self, timeout):
        self.rtabmap_inputs_received = [False, False, False, False]
        startTime = rospy.get_time()
        rospy.loginfo("Waiting to receive inputs for rtabmap...")
        while True:
            count_received = 0
            for input_received in self.rtabmap_inputs_received:
                if input_received:
                    count_received = count_received + 1
            if count_received == 4:
                rospy.loginfo("Inputs received sucessfully")
                return True
            current_duration = rospy.get_time() - startTime
            # rospy.loginfo(current_duration)
            if current_duration > timeout:
                rospy.logerr("Timeout while waiting for rtabmap inputs")
                return False

    def wait_for_rtabmap_outputs(self, timeout):
        self.rtabmap_cloud_received = False
        startTime = rospy.get_time()
        rospy.loginfo("Waiting to receive outputs from rtabmap...")
        while True:
            if self.rtabmap_cloud_received:
                rospy.loginfo("Outputs received sucessfully")
                return True
            current_duration = rospy.get_time() - startTime
            # rospy.loginfo(current_duration)
            if current_duration > timeout:
                rospy.logerr("Timeout while waiting for rtabmap outputs")
                return False
        self.rtabmap_cloud_received

    def control_UR10(self):
        if (self.move_UR10):
            if (self.move_pose == self.UR10_MOVE_HOME_POSE):
                self.move_UR10 = False
                self.ur_move_home()
            elif (self.move_pose == self.UR10_MOVE_FOLD_POSE):
                self.move_UR10 = False
                self.ur_move_fold()
            elif (self.move_pose == self.UR10_MOVE_NEXT_POSE):
                self.move_UR10 = False
                self.ur_move_scan_next_pose()
            elif (self.move_pose == self.UR10_MOVE_SCAN_CONTINUOUS):
                self.ur_move_scan_continuous()
        else:
            if (self.rtabmap_enable and not self.rtabmap_is_paused):
                self.rtabmap_pause()
                self.rtabmap_pause_odom()


if __name__ == '__main__':
    ur10_automap = UR10Automap()
    ur10_automap.spin(100)
