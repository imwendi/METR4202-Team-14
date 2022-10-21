#!/usr/bin/env python3

import rospy
from joint_controller.definitions import *
from fiducial_msgs.msg import FiducialTransformArray
from vision.cube import Cube




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

    def get_closest(self, target_position: np.array, verbose=False):
        """
        Finds cube closest to a target position

        Args:
            target_position: position to compare to

        Returns:
            Cube instance for closest cube

        """
        if len(self.cubes) == 0:
            return None

        closest_cube = None
        displacement = 42069    # haha
        cubes = list(self.cubes.values())
        for cube in cubes:
            new_displacement = np.linalg.norm(cube.avg_pos() - target_position)
            if closest_cube is None or new_displacement < displacement:
                displacement = new_displacement
                closest_cube = cube

        if verbose:
            print(f"Cube {closest_cube.id} closest at {np.around(closest_cube.avg_pos(), 3)}")

        return closest_cube

    def remove_cube(self, id):
        self.cubes.pop(id)

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
