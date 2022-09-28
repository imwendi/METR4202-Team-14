import numpy as np
from kinematics.kinematics_base import KinematicsBase
from kinematics.collision import intersect_connected_segments


class RobotKinematics(KinematicsBase):
    """
    Subclass of KinematicsBase with added collision computation

    """
    def check_self_collision(self, joint_angles):
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

        return intersect_connected_segments(joint_pos)

    def get_planar_joint_pos(self, joint_angles):
        """
        Extends KinematicsBase.joint_pos_3r to compute joint positions in the 3R
        plane also including the base joint.

        Args:
            joint_angles: 3R chain joint angles (excludes first joint)

        Returns:
            list of joint coordinates in the 3R plane
            [p0, p1, p2, p3, p_end], p0 is at the origin

        """
        p0 = np.zeros(2)
        joint_pos = self.joint_pos_3r(self.link_lengths[1:], joint_angles)
        # shift [p1, p2, p3, p_end] relative to p0
        offset = np.array([self.link_lengths[0], 0])
        for i, pos in enumerate(joint_pos):
            joint_pos[i] = pos + offset

        return [p0] + joint_pos