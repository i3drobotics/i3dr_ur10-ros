#!/usr/bin/env python

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Image
from stereo_msgs.msg import DisparityImage
import rospkg
import rospy

from cv_bridge import CvBridge, CvBridgeError

import time
import threading

import cv2
import numpy as np

import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GLib, GObject, GdkPixbuf

rospack = rospkg.RosPack()

test_image = np.zeros((300, 300, 3), np.uint8)
test_image[:] = (0,0,255)

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


class GtkHandler:
	def __init__(self):
		# Set the path to Glade file, in the ros_glade ROS package
		# str=roslib.packages.get_pkg_dir('ros_glade')+"/nodes/ROS_Button.glade"
		self.gladefile = rospack.get_path('i3dr_ur10') + '/ui/roomScanUR10.glade'
		# Initiate the Builder and point it to the glade file
		self.builder = Gtk.Builder()
		self.builder.add_from_file(self.gladefile)
		# Connect event functions
		self.builder.connect_signals(self)
		self.window = self.builder.get_object("wndMain")
		self.window.show_all()

		self.lblUR10BaseAngle = self.builder.get_object("lblUR10BaseAngle")
		self.lblUR10ShoulderAngle = self.builder.get_object("lblUR10ShoulderAngle")
		self.lblUR10ElbowAngle = self.builder.get_object("lblUR10ElbowAngle")
		self.lblUR10Wrist1Angle = self.builder.get_object("lblUR10Wrist1Angle")
		self.lblUR10Wrist2Angle = self.builder.get_object("lblUR10Wrist2Angle")
		self.lblUR10Wrist3Angle = self.builder.get_object("lblUR10Wrist3Angle")

		self.lblScanStatus = self.builder.get_object("lblScanStatus")

		self.imgLeftCamera = self.builder.get_object("imgLeftCamera")
		self.imgRightCamera = self.builder.get_object("imgRightCamera")
		self.imgDisparity = self.builder.get_object("imgDisparity")

		# Initialise ros node
		rospy.init_node('i3dr_ur10_sample_scan_gui')
		rospy.Subscriber("i3dr_scan_status", String,
						 self.i3dr_scan_status_callback)

		rospy.on_shutdown(self.shutdown)

		self.cvbridge = CvBridge()
		self.sub_left_image = rospy.Subscriber("left/image_raw",Image,self.get_left_image)
		self.sub_right_image = rospy.Subscriber("right/image_raw",Image,self.get_right_image)
		self.sub_disparity_image = rospy.Subscriber("disparity",DisparityImage,self.get_disparity_image)

		# Initalise joint angle thread
		joint_thread = threading.Thread(target=self.get_joint_angles, args=())
		joint_thread.daemon = True
		joint_thread.start()

	def shutdown(self):
		Gtk.main_quit()

	def wndMain_onDestroy(self, *args):
		rospy.signal_shutdown("i3dr_ur10_gui window closed")
		self.shutdown()

	def btnFoldPose_onClicked(self, button):
		self.emptySrvClient("i3dr_scan_fold_pose")

	def btnHomePose_onClicked(self, button):
		self.emptySrvClient("i3dr_scan_home_pose")

	def btnNextScanPose_onClicked(self, button):
		self.emptySrvClient("i3dr_scan_next_pose")

	def btnContinuousScan_onClicked(self, button):
		self.emptySrvClient("i3dr_scan_continuous")

	def btnCancelPose_onClicked(self, button):
		self.emptySrvClient("i3dr_scan_cancel_pose")

	def emptySrvClient(self, srv_name):
		if not rospy.is_shutdown():
			print "sending service call"
			rospy.wait_for_service(srv_name, timeout=2)
			try:
				srv = rospy.ServiceProxy(srv_name, Empty)
				resp = srv()
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			print "service call complete"
		else:
			print "node has shutdown"

	def show_left_frame(self, frame):
		self.show_frame(frame,self.imgLeftCamera)

	def show_right_frame(self, frame):
		self.show_frame(frame,self.imgRightCamera)

	def show_disparity_frame(self, frame):
		self.show_frame(frame,self.imgDisparity)

	def show_frame(self, frame, gtkimage):
		frame = cv2.resize(frame, (480,270), interpolation = cv2.INTER_CUBIC)
		pb = GdkPixbuf.Pixbuf.new_from_data(frame.tostring(),
										GdkPixbuf.Colorspace.RGB,
										False,
										8,
										frame.shape[1],
										frame.shape[0],
										frame.shape[2]*frame.shape[1])
		gtkimage.set_from_pixbuf(pb.copy())

	def get_left_image(self,data):
		try:
			cv_image = self.cvbridge.imgmsg_to_cv2(data, "mono8")
			cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
			GLib.idle_add(self.show_left_frame, cv_image)
		except CvBridgeError as e:
			print(e)

	def get_right_image(self,data):
		try:
			cv_image = self.cvbridge.imgmsg_to_cv2(data, "mono8")
			cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
			GLib.idle_add(self.show_right_frame, cv_image)
		except CvBridgeError as e:
			print(e)

	def get_disparity_image(self,data):
		try:
			cv_image = self.cvbridge.imgmsg_to_cv2(data.image,"passthrough")
			cv_image = np.uint8(cv_image)
			cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
			GLib.idle_add(self.show_disparity_frame, cv_image)
		except CvBridgeError as e:
			print(e)
 
	def update_joint_angles(self, joints):
		self.lblUR10BaseAngle.set_text(str(round(joints[0], 2)))
		self.lblUR10ShoulderAngle.set_text(str(round(joints[1], 2)))
		self.lblUR10ElbowAngle.set_text(str(round(joints[2], 2)))
		self.lblUR10Wrist1Angle.set_text(str(round(joints[3], 2)))
		self.lblUR10Wrist2Angle.set_text(str(round(joints[4], 2)))
		self.lblUR10Wrist3Angle.set_text(str(round(joints[5], 2)))

	def get_joint_angles(self):
		loop = True
		while loop:
			if not rospy.is_shutdown():
				joint_states = rospy.wait_for_message("joint_states", JointState)
				joints_pos = joint_states.position
				joints_pos_deg = joints_rad_to_deg(joints_pos)
				GLib.idle_add(self.update_joint_angles,joints_pos_deg)
				# print joints_pos_deg
				time.sleep(0.1)
			else:
				print "node has shutdown"
				loop = False

	def update_scan_status(self,status):
		self.lblScanStatus.set_text(status)

	def i3dr_scan_status_callback(self,data):
		GLib.idle_add(self.update_scan_status,data.data)

if __name__ == "__main__":
	GObject.threads_init()
	gtk_h = GtkHandler()
	Gtk.main()

