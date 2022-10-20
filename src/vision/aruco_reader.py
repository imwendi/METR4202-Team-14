#!/usr/bin/env python3

import rospy
from joint_controller.definitions import *
from fiducial_msgs.msg import FiducialTransformArray
from vision.cube import Cube
from vision.definitions import TRANSFORM_MATRIX




class ArucoReader:
    """
    Class for continuously parsing Aruco data

    """
    def __init__(self):
        # dictionary mapping cube Aruco ids to Cube instances
        self.cubes = {}

        # fiducial subscriber
        self.sub = rospy.Subscriber('/fiducial_transforms',  # Topic name
                                    FiducialTransformArray,  # Message type
                                    self._process_fiducials  # Callback function (required)
                                    )

    def get_closest(self, target_position: np.array) -> Cube:
        """
        Finds cube closest to a target position

        Args:
            target_position: position to compare to

        Returns:
            Cube instance for closest cube

        """
        closest_cube = self.cubes[0]
        displacement = np.linalg.norm(closest_cube.avg_pos() - target_position)
        for cube in self.cubes[1:]:
            new_displacement = np.linalg.norm(cube.avg_pos() - target_position)
            if new_displacement < displacement:
                displacement = new_displacement
                closest_cube = cube

        return closest_cube

    def _process_fiducials(self, fid_array: FiducialTransformArray):
        for fid_transform in fid_array.transforms:
            id = fid_transform.fiducial_id
            transform = fid_transform.transform

            # create new Cube instance for new detected cube
            if id not in self.cubes.keys():
                self.cubes[id] = Cube(id)
            cube = self.cubes[id]

            cube.update(transform)

            if cube.moving:
                print(f"cube {cube.id} moving!")
            else:
                print()
