from vision.aruco_reader import ArucoReader
from robot.motion_controller import MotionController
import numpy as np


class Robot:
    def __init__(self):
        self.motion_controller = MotionController()
        self.aruco_reader = ArucoReader()

    def task1(self):
        # wait for a non-moving cube
        cube = None
        while cube is None:
            target_position = self.motion_controller.last_position
            cube = self.aruco_reader.get_closest(target_position)

            if cube is not None:
                print(f"Cube {cube.id} closest")
            else:
                print()





