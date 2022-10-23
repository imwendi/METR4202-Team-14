#!/usr/bin/python3

from robot.robot import Robot
import numpy as np
import rospy
import time
rospy.init_node('robot')

rob = Robot()

if __name__ == '__main__':
    #rob.motion_controller.move_to_pos(np.array([0, 0, 417]))

    while not rospy.is_shutdown():
        cubes = rob.aruco_reader.cubes.values()
        for cube in cubes:
            data = cube.get_latest_data()
            if data is not None:
                timestamp, position, orientation, moving = data
                print(f"Cube ID: {cube.id}")
                print(f"Time stamp: {timestamp}")
                print(f"Cube position: {np.around(position, 3)}")
                print(f"Cube z orientation: {np.around(np.rad2deg(orientation), 3)} deg")

                print()

                unwrapped_orientation = rob.aruco_reader.unwrap_to_90(orientation)
                print(f"Unwrapped orientation: {np.around(np.rad2deg(unwrapped_orientation), 3)} deg")

                theta = rob.aruco_reader.unwrap_to_90(np.arctan2(position[1], position[0]))
                print(f"Unwrapped theta: {np.around(np.rad2deg(theta), 3)} deg")
                print()

        print('\n' + '_'*50 + '\n')

        time.sleep(0.1)
