from ctypes import Union
from typing import List, Optional

import numpy as np
import time
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Transform
from joint_controller.utils import numpify
from vision.definitions import CAMERA_SCALE
from vision.definitions import T_ROBOT_CAM
from kinematics.utils import apply_transform
from joint_controller.definitions import *


"""
Class used to store data about a single cube

"""
class Cube:
    def __init__(self,
                 id,
                 cache_length=5,
                 avg_length=3,
                 moving_threshold=15):
        """
        Constructor

        Args:
            id: Fiducial id
            cache_length: length of position and z orientation caches
            avg_length: number of values to compute moving average over
            moving_threshold: position delta threshold to determine if cube is moving
        """
        self.id = id
        self.color = None

        # timestamped data about cube movements
        self.data = {
            'timestamp': [],
            'position': [],
            'orientation': [],
            'moving': []
        }

        self.cache_length = cache_length
        self.avg_length = avg_length
        self.moving_threshold = moving_threshold
        self.moving = False
        self.update_time = time.time()
        self.orientation = 0

    def update(self, transform: Transform):
        """
        Callback to update cube pose data from a transform message

        """
        if transform is None:
            return

        timestamp = time.time()

        # scale and transform cube position to robot stationary frame
        position = numpify(transform.translation) * CAMERA_SCALE
        position = apply_transform(T_ROBOT_CAM, position)

        # extract rotation
        quaternion = numpify(transform.rotation)
        euler_angles = euler_from_quaternion(quaternion)
        z_orientation = euler_angles[-1]

        # check if moving
        pos_change = position - self.avg_pos()
        if np.linalg.norm(pos_change) > self.moving_threshold:
            moving = True
        else:
            moving = False

        self.moving = moving
        self.update_time = timestamp
        self.orientation = z_orientation

        # ignore invalid values
        if position is None or np.any(np.isnan(position)):
            return False

        # update data
        self.data['timestamp'].append(timestamp)
        self.data['position'].append(position)
        self.data['orientation'].append(z_orientation)
        self.data['moving'].append(moving)
        self.truncate_caches()

    def position(self):
        """
        Returns:
            latest cube timestamp and position

        """
        if len(self.data['timestamp']) > 0:
            timestamp = self.data['timestamp'][-1]
            position = self.data['position'][-1]

            return timestamp, position

        return [None, None]

    def get_latest_data(self, key=None):
        """
        Args:
            key: key of parameter to extract, else all returned

        Returns:
            Data corresponding to latest update timestamp

        """
        if len(self.data['timestamp']) > 0:
            if key is not None:
                return self.data[key][-1]
            else:
                return [self.data[key][-1]
                        for key in ['timestamp', 'position', 'orientation', 'moving']]

        return None

    def truncate_caches(self):
        """
        Truncates cached data lengths

        """
        for (key, val) in self.data.items():
            if len(val) > self.cache_length:
                self.data[key] = val[-self.cache_length:]

    def avg_pos(self):
        """
        Returns:
            Moving average over last self.avg_pos values

        """
        values = np.array(self.data['position'])[max(-self.avg_length,
                                                     -len(self.data['position'])):-1]

        if np.any(np.isnan(values)):
            print("nan vals found for ", self.data['position'])

        return np.mean(values, axis=0)
