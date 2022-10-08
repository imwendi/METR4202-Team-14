#!/usr/bin/python3

import rospy
from joint_controller.joint_controller import JointController

rospy.init_node('joint_controller')
joint_controller = JointController()

if __name__ == '__main__':
    rospy.spin()
