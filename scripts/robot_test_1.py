#!/usr/bin/python3

from robot.robot import Robot

import rospy
import time
rospy.init_node('robot')

rob = Robot()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        rob.task1()
        time.sleep(0.25)
