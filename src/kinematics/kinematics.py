from typing import Union

import numpy as np
from kinematics.kinematics_base import KinematicsBase
from kinematics.collision import intersect_connected_segments


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

    # ________________________ Joint angles selection __________________________

    def pick_joint_angles(self, ik_solution):
        """
        Computes selects joint angle combination from inverse kinematics solutions
        which best satisfy the following criteria.

        Necessary conditions for valid solutions:
            - No NaN values (occurs when an IK solution isn't valid)

            - No joint positions causing self-collisions
                            (line intersections of links)
            - No joint positions have vertical (z) value below the vertical threshold

        Sorting conditions with which to prioritise valid solutions:
            - Greatest joint 2 vertical (z) value

        Args:
            ik_solution: inverse kinematics solutions from:
                (4, 4) array with joint angle combinations on columns

        Returns:
            selected joint angles, None if no valid solutions are available

        """
        necessary_cond_filters = [self._filter_nan]

        # remove solution columns with NaN values
        ik_solution = self._filter_nan(ik_solution)
        # run through filters
        for filter in necessary_cond_filters:
            ik_solution = filter(ik_solution)

            # no valid solutions if all columns are deleted
            if ik_solution.shape[1] == 0:
                print(f'failed on {filter.__name__}')
                return None

        print('length', len(ik_solution))

        return ik_solution[:, 0]

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

    def _filter_collision(self, ik_solutions: np.array) -> np.array:
        """
        Args:
            ik_solutions: inverse kinematics solutions

        Returns:
            inverse kinematics solutions with solution columns that allow for
            self-collision removed
        """
        num_sols = ik_solutions.shape[1]
        collision_idx = [self.check_self_collision(ik_solutions[:, i])
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

        for i, joint_positions in all_joint_positions:
            if (joint_positions[:, i] > self.vertical_threshold).all():
                threshold_satisfied_idx[i] = True

        return ik_solutions[:, threshold_satisfied_idx]

    def check_self_collision(self, joint_angles, verbose=False):
        """
        Checks if a set of joint angles will result in self collisions

        Args:
            joint_angles: joint angles

        Returns:
            True if a collision will occur, else False

        """
        # extract only the 3R angles if all 4 angles are given
        if len(joint_angles) == 4:
            joint_angles = joint_angles[1:]

        joint_pos = self.get_planar_joint_pos(joint_angles)

        return intersect_connected_segments(joint_pos, verbose=verbose)

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
