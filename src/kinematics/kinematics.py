from typing import Union

import numpy as np
from kinematics.kinematics_base import KinematicsBase
from kinematics.collision import intersect_connected_segments
from joint_controller.definitions import *


class RobotKinematics(KinematicsBase):
    """
    Extends KinematicsBase with higher-level functionality:
        - collision computation
        - optimal joint angle selection from IK solutions

    """
    def __init__(self, *args, vertical_threshold=0, **kwargs):
        """

        Args:
            *args, **kwargs: passed into KinematicsBase's constructor
            vertical_threshold: joint position vertical threshold
        """
        super().__init__(*args, **kwargs)
        self.vertical_threshold = vertical_threshold

    def pick_pose_from_position(self, position, orientation_options=None):
        """
        Given desired end-affector position (x, y, z), computes valid orientation
        and valid joint angles to reach that position.

        Args:
            position: end-affector position
            orientation_options: array of 3R orientation angles to search through

        Returns:
            joint_angles: chosen joint angles or None to position cannot be reached
            chosen_orientation: chosen 3R planar orientation angle

        """
        joint_angles = None
        chosen_orientation = None

        if orientation_options is None:
            orientation_options = np.concatenate([
                # np.linspace(180, 90, 50),
                # np.linspace(90, -180, 50)
                np.linspace(90, 180, 50),
                np.linspace(90, 0, 50)
            ])

        orientation_options = np.deg2rad(orientation_options)

        # TODO: delete print
        #print(orientation_options.shape)
        for i, orientation in enumerate(orientation_options):
            print(i)
            ik_solutions = self.ik(position, orientation)
            ik_solutions = self.filter_ik_solution(ik_solutions)

            if ik_solutions is None:
                continue
            else:
                joint_angles = self.pick_highest_joint_2(ik_solutions)
                chosen_orientation = orientation
                break

        return joint_angles, chosen_orientation

    def pick_highest_joint_2(self, ik_solution):
        """

        Args:
            ik_solution: inversion kinematics solution

        Returns:

        """
        # joint positions (joint angle combination, [x, y, z] coordinate, joint number)
        ik_joint_positions = self.get_ik_joint_positions(ik_solution)

        # second last joint at idx 2, z axis is at idx 2
        # nanargmax ignores NaN values
        target_idx = np.nanargmax(ik_joint_positions[:, 2, 2])

        return ik_solution[:, target_idx]

    def get_ik_joint_positions(self, ik_solution):
        """

        Args:
            ik_solution: inverse kinematics solution

        Returns:
            3d array of joint positions, dimensions are:
             (joint angle combination, [x, y, z] coordinate, joint number)

        """
        num_sols = ik_solution.shape[1]

        return np.array([self.joint_pos(ik_solution[:, i])
                         for i in range(num_sols)])

    # _____________________________ IK Filtering _______________________________

    def filter_ik_solution(self, ik_solution):
        """
        Filters inverse kinematics solutions for joint angle combintations
        which satisfy the following necessary conditions:

        - No NaN values (occurs when an IK solution isn't valid)

        - No joint positions causing self-collisions
                        (line intersections of links)
        - No joint positions have vertical (z) value below the vertical threshold

        Args:
            ik_solution: inverse kinematics solutions from:
                (4, 4) array with joint angle combinations on columns

        Returns:
            selected joint angles, None if no valid solutions are available

        """
        necessary_cond_filters = [self._filter_nan,
                                  self._filter_dynamixel_limits,
                                  self._filter_collision,
                                  self._filter_vertical_threshold]

        # remove solution columns with NaN values
        ik_solution = self._filter_nan(ik_solution)
        # run through filters
        for filter in necessary_cond_filters:
            ik_solution = filter(ik_solution)

            # no valid solutions if all columns are deleted
            if ik_solution.shape[1] == 0:
                print(f'failed on {filter.__name__}')
                return None

        return ik_solution

    def _filter_nan(self, ik_solutions: np.array) -> np.array:
        """
        Args:
            ik_solutions: inverse kinematics solutions

        Returns:
            inverse kinematics solutions with solution columns with
            NaN values removed

        """
        nan_idx = np.isnan(ik_solutions).any(axis=0)
        ik_solutions = np.delete(ik_solutions, nan_idx, axis=1)

        return ik_solutions

    def _filter_dynamixel_limits(self, ik_solutions: np.array) -> np.array:
        """
        Args:
            ik_solutions: ik_solutions: inverse kinematics solutions

        Returns:
            inverse kinematics solutions with solution columns with
            invalid angles removed

        """
        # min_mask = (DYNAMIXEL_MIN_ANGLE < ik_solutions)
        # max_mask = (DYNAMIXEL_MAX_ANGLE > ik_solutions)
        #
        # select_idx = np.logical_and(min_mask, max_mask).all(axis=0)

        print('dynamixel limits shape', DYNAMIXEL_ANGLE_LIMS.shape)
        print('ik_solutions shape ', ik_solutions.shape)

        mask = np.array((np.abs(ik_solutions) < DYNAMIXEL_ANGLE_LIMS))

        print('mask ', mask)
        print('mask shape ', mask.shape)

        mask = mask.all(axis=0)

        # print('min_mask ', min_mask)
        # print('max_mask ', max_mask)
        print('select_idx ', mask)

        ik_solutions = ik_solutions[:, mask]

        print('selected ik solutions ', ik_solutions)

        return ik_solutions

    def _filter_collision(self, ik_solutions: np.array) -> np.array:
        """
        Args:
            ik_solutions: inverse kinematics solutions

        Returns:
            inverse kinematics solutions with solution columns that allow for
            self-collision removed
        """
        num_sols = ik_solutions.shape[1]
        collision_idx = [self.check_self_collision(ik_solutions[:, i], verbose=True, skip_consecutive=True)
                         for i in range(num_sols)]
        ik_solutions = np.delete(ik_solutions, collision_idx, axis=1)

        return ik_solutions

    def _filter_vertical_threshold(self, ik_solutions: np.array) -> np.array:
        """
        Args:
            ik_solutions: inverse kinematics solutions

        Returns:
            inverse kinematics solutions with solutions columns with joint
            positions not satisfying the vertical threshold removed.

        """
        num_sols = ik_solutions.shape[1]
        # all joint positions
        all_joint_positions = [self.joint_pos(ik_solutions[:, i])
                               for i in range(num_sols)]
        threshold_satisfied_idx = np.full(num_sols, False)


        for i, joint_positions in enumerate(all_joint_positions):
            # row 2 is z coordinate values
            if (joint_positions[2, :] > self.vertical_threshold).all():
                threshold_satisfied_idx[i] = True

        return ik_solutions[:, threshold_satisfied_idx]

    def check_self_collision(self, joint_angles, **kwargs):
        """
        Checks if a set of joint angles will result in self collisions

        Args:
            joint_angles: joint angles
            **kwargs: kwargs to pass into intersect_connected_segments

        Returns:
            True if a collision will occur, else False

        """
        # extract only the 3R angles if all 4 angles are given
        if len(joint_angles) == 4:
            joint_angles = joint_angles[1:]

        joint_pos = self.get_planar_joint_pos(joint_angles)

        return intersect_connected_segments(joint_pos, **kwargs)

    def get_planar_joint_pos(self, joint_angles):
        """
        Extends KinematicsBase.joint_pos_3r to compute joint positions in the 3R
        plane also including the base joint.

        Args:
            joint_angles: 3R chain joint angles (excludes first joint)

        Returns:
            list of coordinates (ground + joints) in the 3R plane
            [ground, p0, p1, p2, p3, p_end], p0 is at the origin

        """
        ground = np.zeros(2)
        joint_pos = self.joint_pos_3r(self.link_lengths[1:], joint_angles)
        p0 = np.array([self.base_height, 0])
        # shift [p1, p2, p3, p_end] relative to p0
        offset = np.array([self.link_lengths[0], 0]) + p0
        for i, pos in enumerate(joint_pos):
            joint_pos[i] = pos + offset

        return [ground, p0] + joint_pos
