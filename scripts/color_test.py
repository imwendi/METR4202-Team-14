#!/usr/bin/python3

import rospy
from vision.aruco_reader import ArucoReader
import time

rospy.init_node("own_color_test")

ar = ArucoReader()


if __name__ == '__main__':
    #ar.calibrate_empty_color()
    while not rospy.is_shutdown():
        print('current color: ', ar.identify_color())
        time.sleep(0.1)
