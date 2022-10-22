#!/usr/bin/env python3
import time

import numpy as np
import rospy
from joint_controller.definitions import *
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import ColorRGBA
from vision.definitions import *
from vision.cube import Cube
import vision.color_utils as color_utils


class ArucoReader:
    """
    Class for continuously parsing Aruco data and camera color data

    """
    def __init__(self,
                 color_cache_len=20):
        # dictionary mapping cube Aruco ids to Cube instances
        self.cubes = {}

        # fiducial subscriber
        self.fid_sub = rospy.Subscriber('/fiducial_transforms',  # Topic name
                                        FiducialTransformArray,  # Message type
                                        self._process_fiducials  # Callback function (required)
                                        )

        # list storing historical color data
        self.color_cache_len = color_cache_len
        self.color_cache = [np.zeros(3)]

        # RGB color subscriber
        self.rgb_sub = rospy.Subscriber(NODE_RGB_COLOR,
                                        ColorRGBA,
                                        self._process_color
                                        )

        # color dictionary mapping to EMPTY, RED, GREEN, BLUE, YELLOW
        self.color_map = {
            'red': RED,
            'blue': BLUE,
            'green': GREEN,
            'yellow': YELLOW,
            'no_cube': np.zeros(3)
        }

    def reset(self):
        self.cubes = {}

    def calibrate_empty_color(self):
        time.sleep(2)
        self.color_map['no_cube'] = self.avg_color(5)

    def turntable_empty(self):
        """
        Returns:
            If a cube was recently found and its values updated

        """
        curr_time = time.time()
        for cube in self.cubes.values():
            timestamp, _, _, _ = cube.get_latest_data()
            if abs(curr_time - timestamp) < RECENT_INTERVAL:
                # turntable assumed moving if a cube has moved recently
                return False

        return True

    def turntable_moving(self):
        """
        Returns:
            If the turntable is moving

        """
        curr_time = time.time()
        for cube in self.cubes.values():
            timestamp, _, _, moving = cube.get_latest_data()
            if abs(curr_time - timestamp) < RECENT_INTERVAL and moving:
                # turntable assumed moving if a cube has moved recently
                return True

        return False


    def identify_color(self, avg_len=5):
        avg_color = self.avg_color(avg_len=avg_len)

        # TODO: remove!
        # print('latest color ', self.color_cache[-1])
        # print('avg color ', avg_color)

        return self.closest_color(avg_color)

    def closest_color(self, color_rgb):
        """
        Searches self.color_map for color closest to a given color

        Args:
            color_rgb: given color in unnormalized RGB

        Returns:
            closest color name (keys of self.color_map)

        """
        closest_color_name = None
        last_distance = np.Infinity

        for (name, color) in self.color_map.items():
            distance = color_utils.hsv_distance(color_rgb, color)
            if closest_color_name is None or distance < last_distance:
                closest_color_name = name
                last_distance = distance

        return closest_color_name

    def avg_color(self, avg_len=5):
        """


        Args:
            avg_len:

        Returns:

        """
        values = np.array(self.color_cache)[max(-avg_len, -len(self.color_cache)):-1]

        return np.mean(values, axis=0)

    def _process_color(self, rgb_color: ColorRGBA):
        color = np.array([rgb_color.r, rgb_color.g, rgb_color.b])
        self.color_cache.append(color)

        # TODO: remove
        # print("added color ", color)

        # truncate list if needed
        if len(self.color_cache) > self.color_cache_len:
            self.color_cache = self.color_cache[-self.color_cache_len:]

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

            # if cube.moving:
            #     print(f"cube {cube.id} moving!")
            # else:
            #     print()


if __name__ == '__main__':
    # sanity check
    ar = ArucoReader()

    # should print red, blue, green, yellow
    print(ar.closest_color(RED)*1.05)
    print(ar.closest_color(BLUE)*1.05)
    print(ar.closest_color(GREEN)*1.05)
    print(ar.closest_color(YELLOW)*1.05)
