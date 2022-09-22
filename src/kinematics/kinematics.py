from typing import Union, List
import numpy as np
import modern_robotics as mr
from kinematics.utils import rot, transform


class RobotKinematics():
    """
    Class for computing kinematics of the project robot.
    Note that we decompose the robot into joint 1 and the planar
    3R chain formed by links 2-4. We denote the latter's plane as the "3R plane"

    """
    def __init__(self, link_lengths: Union[np.array, List[float]]):
        self.link_lengths = link_lengths

    def ik_closest_norm(self, p, pose):
        """
        Computes inverse kinematics and selects pose closest (least l1 norm)
        from a given current pose.

        Args:
            p: desired end-affector position (x, y, z)
            pose: current pose (joint angle array)

        Returns:
            new optimal pose

        """
        new_poses = self.ik(p)
        norm_val = np.linalg.norm(new_poses - pose.reshape((3, 1)))
        pose_idx = np.argmin(norm_val, axis=-1)

        return new_poses[pose_idx]

    def ik(self, p):
        """
        Computes inverse kinematics

        Args:
            p: desired end-affector position (x, y, z)

        Returns:
            possible joint angles (poses) to get to p

        """
        x, y, z, phi = p
        #phi = np.arctan2(z - self.link_lengths[0], np.sqrt(x**2 + y**2))
        theta1 = np.arctan2(*p[:2]) + np.array([0, np.pi])

        poses = np.zeros((4, 8))
        for i, _theta1 in enumerate(theta1):
            # inverse kinematics for 3R chain
            R = rot(0, 0, _theta1)  # rotation to 3R plane
            p_3r = [*(R@p)[:2], phi]   # end-affector position in 3R plane
            _joint_angles = self.ik_3r(p_3r, self.link_lengths[1:])
            # append _theta1 to _joint_angles (theta2-4 values)
            poses[:, i*4:(i+1)*4] =\
                np.concatenate((np.ones((1, 4))*_theta1, _joint_angles))

        return poses

    def joint_pos(self, pose):
        # transformation matrix, 3R to stationary frame
        T = self.transform_to_3r(self.link_lengths[0], pose[0])

        # compute joint positions in 3R plane, (2, 4) initially
        pose_3r = np.array(self.joint_pos_3r(self.link_lengths[1:], pose[1:])).T

        # append 1s to x position and last row for compatibility with T
        # final shape is (4, 4), each column is a joint position (x, y, z, 1)
        pose_3r = np.concatenate([pose_3r, np.zeros((1, 4)), np.ones((1, 4))],
                                 axis=0)

        # convert positions to stationary frame, discard 1s row
        pose_3r = (T @ pose_3r)[:-1, :]

        # origin at (0, 0, 0)
        origin = np.zeros((3, 1))

        # combine positions
        return np.concatenate([origin, pose_3r], axis=-1)

    # ______________________________ 3R methods ________________________________
    @staticmethod
    def transform_to_3r(l1, theta1):
        """
        Compute transformation to 3R plane

        Args:
            l1: first link length
            theta1: first joint angle in pose

        Returns:

        """
        p = np.array([0, 0, l1])
        R = rot(0, 0, theta1) @ rot(0, -np.pi / 2, 0)

        return mr.RpToTrans(R, p)


    @staticmethod
    def ik_3r(link_lengths, p):
        """
        Computes 2D inverse kinematics of 3R planar chain.

        Args:
            link_lengths: link lengths (link 1, link 2, link 3)
            p: desired end-affector position (x, y, orientation)

        Returns:
            joint angles (pose) for joints 1-3, shape (3, 4),
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

            joint_positions.append(joint_positions[i] + link_lengths[i]*pos_delta)

        return joint_positions

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
