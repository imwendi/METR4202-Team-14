#!/usr/bin/env python3

import rospy
from joint_controller.definitions import *
from fiducial_msgs.msg import FiducialTransformArray
from vision.cube import Cube


# camera frame to robot base frame
TRANSFORM_MATRIX = np.array([[0, 1, 0, 190],
                             [1, 0, 0, 0],
                             [0, 0, -1, 426],
                             [0, 0, 0, 1]])

# scale from camera to metric units
SCALE = 450/2.42


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
