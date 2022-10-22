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
        turntable_empty = rob.aruco_reader.turntable_empty()
        turntable_moving = rob.aruco_reader.turntable_moving()
        print(f"Turntable empty: {turntable_empty}")
        print(f"Turntable moving: {turntable_moving}")
        print('_'*50 + '\n')

        # rob.task1()
        time.sleep(0.25)
