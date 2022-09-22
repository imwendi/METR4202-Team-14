from typing import Union, List
import numpy as np
from utils import rot


class RobotKinematics():
    """
    Class for computing kinematics of the project robot.
    Note that we decompose the robot into joint 1 and the planar
    3R chain formed by links 2-4. We denote the latter's plane as the "3R plane"

    """
    def __init__(self, link_lengths: Union[np.array, List[float]]):
        self.link_lengths = link_lengths

    def compute_closest_norm(self, p, pose):
        """
        Computes inverse kinematics and selects pose closest (least l1 norm)
        from a given current pose.

        Args:
            p: desired end-affector position (x, y, z)
            pose: current pose (joint angle array)

        Returns:
            new optimal pose

        """
        new_poses = self.compute(p)
        norm_val = np.linalg.norm(new_poses - pose.reshape((3, 1)))
        pose_idx = np.argmin(norm_val, axis=-1)

        return new_poses[pose_idx]

    def compute(self, p):
        """
        Computes inverse kinematics

        Args:
            p: desired end-affector position (x, y, z)

        Returns:
            possible joint angles (poses) to get to p

        """
        x, y, z = p
        phi = np.arctan2(z - self.link_lengths[0], np.sqrt(x**2 + y**2))
        theta1 = np.arctan2(*p[:2]) + np.array([0, np.pi])

        poses = np.zeros((4, 8))
        for i, _theta1 in enumerate(theta1):
            # inverse kinematics for 3R chain
            R = self.rot_to_3r(_theta1)  # rotation to 3R plane
            p_3r = [*(R@p)[:2], phi]   # end-affector position in 3R plane
            _joint_angles = self.ik_3r(p_3r, self.link_lengths[1:])
            # append _theta1 to _joint_angles (theta2-4 values)
            poses[:, i*4:(i+1)*4] =\
                np.concatenate((np.ones((1, 4))*_theta1, _joint_angles))

        return poses

    @staticmethod
    def ik_3r(link_lengths, p):
        """
        Computes 2D inverse kinematics of 3R planar chain.

        Args:
            link_lengths: link lengths (link 1, link 2, link 3)
            p: desired end-affector position (x, y, orientation)

        Returns:
            joint angles for joints 1-3, shape (3, 4),
            angle combinations on columns

        """
        x, y, phi = p
        l1, l2, l3 = link_lengths
        options = np.array([-1, 1])
        # pre-compute trig values
        s_phi, c_phi = np.sin(phi), np.cos(phi)

        # compute possible theta2 values
        c_theta2 = (x**2 + y**2 + l3**2 - 2*l3*(c_phi*x + s_phi*y) - l1**2 - l2**2)\
                   / (2*l1*l2)
        theta2 = options * np.arccos(c_theta2)

        # compute possible theta3 values
        a, b = x - l3*c_phi, y - l3*s_phi
        c = (a**2 + b**2 + l1**2 - l2**2) / (2*l1)
        theta1 = np.arctan2(a, b) +\
                 options*np.arctan2(c, np.sqrt(a**2 + b**2 - c**2))

        # find all combinations of theta2, theta3 values
        theta = RobotKinematics.array_combine(theta1, theta2)
        theta3 = phi - np.sum(theta, axis=0, keepdims=True)
        theta = np.concatenate((theta, theta3), axis=0)

        return theta

    @staticmethod
    def joint_pos_3r(link_lengths, joint_angles):
        """
        Args:
            link_lengths: 3R link lengths
            joint_angles: 3R chain joint angles

        Returns:
            list of joint coordinates in the 3R plane
            [p0, p1, p2, p_end]

        """
        joint_positions = [np.zeros(2)]
        total_angle = 0
        for i in range(3):
            total_angle += joint_angles[i]
            pos_delta = np.array([np.cos(total_angle), np.sin(total_angle)])

            joint_positions.append(joint_positions[i] +link_lengths[i]*pos_delta)

        return joint_positions

    @staticmethod
    def rot_to_3r(theta1):
        """
        Args:
            theta1: joint 1 angle

        Returns:
            rotation matrix from stationary frame to the 3R plane

        """
        return rot(0, 0, theta1)

    @staticmethod
    def array_combine(a, b):
        """
        Computes all combinations of values in two arrays

        Args:
            a: first array, shape (n,)
            b: second array, shape (n,)

        Returns:
            (2, n) matrix, value combinations on columns

        """
        return np.array(np.meshgrid(a, b)).reshape((2, len(a)*len(b)))
