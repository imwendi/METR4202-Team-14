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
