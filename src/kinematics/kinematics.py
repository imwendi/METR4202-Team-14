import numpy as np
from kinematics.kinematics_base import KinematicsBase
from kinematics.collision import intersect_connected_segments


class RobotKinematics(KinematicsBase):
    """
    Subclass of KinematicsBase with added collision computation

    """
    def ik_best_choice(self):
        """
        Computes inverse kinematics and selects joint angle combination which
        best satisfies the following criteria:


        Args:
            p: desired end-affector position (x, y, z)
            phi: desired 3R orientation TODO: replace this with something better

        Returns:
            possible joint angles to get to p

        """
        pass

    # _____________________ Joint angle criteria helpers _______________________

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
