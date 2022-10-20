#!/usr/bin/python3

import rospy
from vision.aruco_reader import ArucoReader

rospy.init_node("aruco_reader_test")

aruco_reader = ArucoReader()

if __name__ == '__main__':
    rospy.spin()
