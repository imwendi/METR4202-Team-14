#!/usr/bin/python3

from robot.robot import Robot
import numpy as np
import rospy
import time
rospy.init_node('robot')

rob = Robot()

if __name__ == '__main__':
    rob.motion_controller.move_to_pos(np.array([0, 0, 417]))
    while not rospy.is_shutdown():
        rob.task1()
        time.sleep(0.2)
