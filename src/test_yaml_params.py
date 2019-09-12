#!/usr/bin/env python

import rosparam
import rospy

class TestYamlParams:
    def __init__(self):
        self.setupNode()

    def setupNode(self):

        rospy.init_node("test_yaml_params", anonymous=True,
                        disable_signals=True)
        self.ns = rospy.get_namespace()
        self.yaml_file = rospy.get_param('~yaml_file')
        self.yaml_param_data = rosparam.load_file(self.yaml_file)
        rospy.set_param("joint_params",self.yaml_param_data)
        self.yaml_data = rospy.get_param('joint_params')
        self.yaml_single_data = self.yaml_data[0][0]["joints_home"]
        print(self.yaml_data)
        print(self.yaml_single_data)

    def spin(self, rate_hz):
        rate = rospy.Rate(rate_hz)
        try:
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

if __name__ == '__main__':
    testYP = TestYamlParams()
    testYP.spin(100)
