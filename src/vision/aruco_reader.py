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
        }

    def turntable_empty(self):
        """
        Returns:
            If a cube was recently found and its values updated
            i.e. if the turntable has at least one cube on it and is not empty

        """
        curr_time = time.time()
        cubes = list(self.cubes.values())
        for cube in cubes:
            if abs(curr_time - cube.update_time)\
                    < RECENT_INTERVAL:
                # turntable assumed moving if a cube has moved recently
                return False

        return True

    def turntable_moving(self):
        """
        Returns:
            If the turntable is moving
        """
        curr_time = time.time()
        cubes = list(self.cubes.values())
        for cube in cubes:
            if abs(curr_time - cube.update_time) < RECENT_INTERVAL and cube.moving:
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
        Computes average historical RGB color

        Args:
            avg_len: number of historical values over which to take average

        Returns:
            Average RGB color

        """
        values = np.array(self.color_cache)[max(-avg_len, -len(self.color_cache)):-1]

        return np.mean(values, axis=0)

    def _process_color(self, rgb_color: ColorRGBA):
        """
        Callback to save latest Ximea RGB color node data to RGB

        """
        color = np.array([rgb_color.r, rgb_color.g, rgb_color.b])
        self.color_cache.append(color)

        # truncate list if needed
        if len(self.color_cache) > self.color_cache_len:
            self.color_cache = self.color_cache[-self.color_cache_len:]

    def get_closest(self, target_position: np.array):
        """
        Finds cube closest to a target position

        Args:
            target_position: position to compare to

        Returns:
            Cube instance for closest cube

        """
        current_time = time.time()

        if len(self.cubes) == 0:
            return None

        closest_cube = None
        displacement = 42069    # haha
        position = None
        #cubes = list(self.cubes.values())
        cube_ids = list(self.cubes.keys())
        for cube_id in cube_ids:
            cube = self.cubes[cube_id]
            new_displacement = np.linalg.norm(cube.avg_pos() - target_position)

            print("new displacement ", new_displacement)
            print("displacement ", displacement)
            if not np.isnan(new_displacement) and\
                    (closest_cube is None or new_displacement < displacement):
                if np.abs(current_time - cube.update_time) < CUBE_TIMEOUT:
                    displacement = new_displacement
                    closest_cube = cube
                else:
                    print("cube time out!")

        return closest_cube

    def get_closest_orientation(self):
        """
        Finds cube with orientation closest to a target position

        Returns:
            Cube instance for closest cube

        """
        current_time = time.time()

        if len(self.cubes) == 0:
            return None

        closest_cube = None
        orientation_difference = 42069    # haha
        chosen_cube_orientation = None
        chosen_robot_orientation = None

        cube_ids = list(self.cubes.keys())
        for cube_id in cube_ids:
            cube = self.cubes[cube_id]
            cube_position = cube.avg_pos()

            if cube_position is None or np.any(np.isnan(cube_position)):
                return None

            required_robot_orientation = self.unwrap_to_90(np.arctan2(
               cube_position[1], cube_position[0]))
            cube_orientation = self.unwrap_to_90(cube.orientation)

            # unwrapping large value further unwrapping
            if cube_orientation > np.deg2rad(80):
                cube_orientation = np.pi / 2 - cube_orientation

            if required_robot_orientation > np.deg2rad(80):
                required_robot_orientation = np.pi / 2 - required_robot_orientation

            new_orientation_difference = np.abs(required_robot_orientation - cube_orientation)

            if (closest_cube is None or new_orientation_difference < orientation_difference):
                if np.abs(current_time - cube.update_time) < CUBE_TIMEOUT:
                    orientation_difference = new_orientation_difference
                    closest_cube = cube

                    chosen_cube_orientation = cube_orientation
                    chosen_robot_orientation = required_robot_orientation
                else:
                    print("cube time out!")

        if closest_cube is not None:
            print(f"Chosen Cube {closest_cube.id}")
            print(f"cube orientation {np.rad2deg(chosen_cube_orientation)}")
            print(f"robot orientation {np.rad2deg(chosen_robot_orientation)}")


        return closest_cube

    def unwrap_to_90(self, angle):
        return angle - np.floor(angle / (np.pi / 2)) * np.pi / 2

    # def unwrap_to_90(self, angle):
    #     return angle - np.floor(angle / (np.pi / 2)) * np.pi / 2

    def remove_cube(self, id):
        """
        Remove a stored cube's data

        Args:
            id: ID of cube to remove

        """
        self.cubes.pop(id)

    def _process_fiducials(self, fid_array: FiducialTransformArray):
        """
        Callback for updating self.cubes data from new Fiducial array data
        from the camera

        """
        for fid_transform in fid_array.transforms:
            id = fid_transform.fiducial_id
            transform = fid_transform.transform

            # create new Cube instance for new detected cube
            if id not in self.cubes.keys():
                self.cubes[id] = Cube(id)
            cube = self.cubes[id]

            cube.update(transform)

    def reset(self):
        """
        Resets all stored cube data

        """
        self.cubes = {}


if __name__ == '__main__':
    # sanity check
    ar = ArucoReader()

    # should print red, blue, green, yellow
    print(ar.closest_color(RED)*1.05)
    print(ar.closest_color(BLUE)*1.05)
    print(ar.closest_color(GREEN)*1.05)
    print(ar.closest_color(YELLOW)*1.05)
