import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Transform
from joint_controller.utils import numpify
import rospy

from joint_controller.definitions import *


class Cube:
    def __init__(self,
                 id,
                 cache_length=50,
                 avg_length=10,
                 moving_threshold=10):
        """
        Constructor

        Args:
            id: Fiducial id
            cache_length: length of position and z orientation caches
            avg_length: number of values to compute moving average over
            moving_threshold: position delta threshold to determine if cube is moving
        """
        self.id = id
        # cube positions
        self.positions = []
        # cube z orientations
        self.orientations = []
        self.cache_length = cache_length
        self.avg_length = avg_length
        self.moving_threshold = moving_threshold
        self.moving = False

    def update(self, transform: Transform):
        if transform is None:
            return

        # extract
        position = numpify(transform.translation)
        quaternion = numpify(transform.rotation)
        euler_angles = euler_from_quaternion(quaternion)
        z_orientation = euler_angles[-1]

        # check if moving
        pos_change = position - self.average_pos()
        if np.linalg.norm(pos_change) > self.moving_threshold:
            self.moving = True
        else:
            self.moving = False

        self.positions.append(position)
        self.orientations.append(z_orientation)

        print(f"Cube {self.id}: {np.around(np.rad2deg(z_orientation), 2)} deg")

        # truncate lists if needed
        if (len(self.positions) > self.cache_length):
            self.positions = self.positions[-self.cache_length:]
        if (len(self.orientations) > self.cache_length):
            self.orientations = self.orientations[-self.cache_length:]

    def average_pos(self):
        """
        Returns:
            Moving average over last self.avg_pos values
        """
        values = np.array(self.positions)[:max(-self.avg_length, len(self.positions))]

        # TODO: check axis!
        return np.mean(values, axis=0)
