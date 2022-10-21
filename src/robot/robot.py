from vision.aruco_reader import ArucoReader
from robot.motion_controller import MotionController
from robot.definitions import *
import numpy as np


class Robot:
    def __init__(self):
        self.motion_controller = MotionController()
        self.aruco_reader = ArucoReader()

    def task1(self):
        # wait for a non-moving cube
        cube = None
        target_pos = None
        while cube is None:
            target_position = self.motion_controller.last_position
            cube = self.aruco_reader.get_closest(target_position)

            if cube is not None:
                target_pos = cube.avg_pos()
            else:
                print()

        target_pos[-1] = FOLLOW_HEIGHT
        self.motion_controller.move_to_pos(cube.avg_pos())






