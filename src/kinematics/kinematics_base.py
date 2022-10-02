from typing import Union, List
import numpy as np
import modern_robotics as mr
from kinematics.utils import rot, transform, apply_transform


class KinematicsBase():
    """
    Class for computing kinematics of the project robot.
    Note that we decompose the robot into joint 1 and the planar
    3R chain formed by links 2-4. We denote the latter's plane as the "3R plane"

    """
    def __init__(self,
                 base_height: float,
                 link_lengths: Union[np.array, List[float]]):
        """
        Constructor

        Args:
            base_height: height from ground of first joint
            link_lengths: lengths of links between joints
        """
        self.base_height = base_height
        self.link_lengths = link_lengths

        # computes transformation matrix to 3R plane
        self.T_3r = lambda theta1:\
            self.transform_from_3r(base_height + link_lengths[0], theta1)

        # space home config.
        self.M = np.eye(4); self.M[2, 3] = np.sum(base_height + link_lengths)

        # forward kinematics space screws, as matrix columns
        l0 = base_height
        l1, l2, l3, l4 = self.link_lengths
        self.screws = np.array([[0, 0, 1, 0, 0, 0],
                                [1, 0, 0, 0, l0+l1, 0],
                                [1, 0, 0, 0, l0+l1+l2, 0],
                                [1, 0, 0, 0, l0+l1+l2+l3, 0]]).T

    # def ik_closest_norm(self, p, phi, thetas):
    #     """
    #     TODO: depreciated?
    #
    #     Computes inverse kinematics and selects pos closest (least l1 norm)
    #     from a given current pos.
    #
    #     Args:
    #         p: desired end-affector position (x, y, z)
    #         phi: desired orientation in 3R plane TODO: for testing only
    #         thetas: current joint angles
    #
    #     Returns:
    #         new optimal pos
    #
    #     """
    #     new_joint_angles = self.ik(p, phi)
    #
    #     norm_val = np.linalg.norm(new_joint_angles - thetas.reshape((4, 1)))
    #     pose_idx = np.argmin(norm_val, axis=-1)
    #
    #     return new_joint_angles[:, pose_idx]

    def fk(self, joint_angles) -> np.array:
        """
        Space forward kinematics for given set of joint angles

        Args:
            joint_angles: configuration joint angles

        Returns:
            Transformation matrix from home config

        """

        return mr.FKinSpace(self.M, self.screws, joint_angles)

    def ik(self, p, phi):
        """
        Computes inverse kinematics

        Args:
            p: desired end-affector position (x, y, z)
            phi: desired 3R orientation TODO: replace this with something better

        Returns:
            possible joint angles to get to p

        """
        theta1 = np.arctan2(p[1], p[0]) + np.array([0, np.pi])

        joint_angles = np.zeros((4, 4))
        for i, _theta1 in enumerate(theta1):
            # transformation to 3R plane
            T = self.T_3r(_theta1)

            # end-affector (x, y, phi) position in 3R plane
            p_3r = [*apply_transform(mr.TransInv(T), p)[:2], phi]
            _joint_angles = self.ik_3r(self.link_lengths[1:], p_3r)

            # append _theta1 to _joint_angles (theta2-4 values)
            joint_angles[:, i*2:(i+1)*2] =\
                np.concatenate((np.ones((1, 2))*_theta1, _joint_angles), axis=0)

        return joint_angles

    # TODO: old, remove?
    # def joint_pos_fk(self, joint_angles):
    #     """
    #     Forward kinematics based joint position computation
    #
    #     Args:
    #         joint_angles: configuration joint angles
    #
    #     Returns:
    #         list of joint coordinates in the stationary plane
    #         [p0, p1, p2, p3, p_end], p0 is at the origin
    #
    #     """
    #     joint_pos = [np.zeros(3)]*5
    #
    #     l1, l2, l3, l4 = self.link_lengths
    #
    #     # compute each positions of joints 1, 2, 3, 4, tip
    #     M = np.eye(4)
    #     curr_pos = np.array([0, 0, self.base_height])
    #     for i, l in enumerate(self.link_lengths):
    #         M[2, 3] += l
    #         T = mr.FKinSpace(M, self.screws[:, :i+1], joint_angles[:i+1])
    #         # joint position in home configuration
    #         joint_pos[i+1] = apply_transform(T, curr_pos)
    #         curr_pos[-1] += l
    #
    #
    #     return joint_pos

    def joint_pos(self, joint_angles):
        """
        Geometry based joint position computation

        Args:
            joint_angles: configuration joint angles

        Returns:
            list of joint coordinates in the stationary plane
            [p0, p1, p2, p3, p_end], p0 is at the base

        """
        # transformation matrix from 3R to stationary frame
        T = self.T_3r(joint_angles[0])

        # compute joint positions in 3R plane, (2, 4) initially
        pos_3r = np.array(self.joint_pos_3r(self.link_lengths[1:], joint_angles[1:])).T

        # append 0s to z row
        # final shape is (3, 4), each column is a joint position (x, y, z)
        pos_3r = np.concatenate([pos_3r, np.zeros((1, 4))], axis=0)

        # convert positions to stationary frame
        pos_3r = apply_transform(T, pos_3r)

        # joint 1 at (0, 0, base_height)
        joint_1 = np.array([0, 0, self.base_height]).reshape((3, 1))

        # combine positions
        return np.concatenate([joint_1, pos_3r], axis=-1)

    # ______________________________ 3R methods ________________________________
    @staticmethod
    def transform_from_3r(displacement, theta1):
        """
        Compute transformation to 3R plane

        Args:
            displacement: vertical displacement of first joint from ground
            theta1: first joint angle

        Returns:
            Transformation matrix to 3R plane

        """

        return mr.RpToTrans(rot(-np.pi / 2, -np.pi / 2, theta1), np.array([0, 0, displacement]))

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

        print("cos val: ", c_phi)

        # compute possible theta2 values
        c_theta2 = (x**2 + y**2 + l3**2 - 2*l3*(c_phi*x + s_phi*y) - l1**2 - l2**2)\
                   / (2*l1*l2)
        theta2 = options * np.arccos(c_theta2)

        # compute possible theta3 values
        a, b = x - l3*c_phi, y - l3*s_phi
        c = (a**2 + b**2 + l1**2 - l2**2) / (2*l1)

        theta1 = np.arctan2(b, a) + \
                 -options * np.arctan2(np.sqrt(a**2 + b**2 - c**2), c)
        # equivaelent alternative computation
        #theta1 = np.arctan2(x, y) - np.arctan2(l2*np.sin(theta2), l1 + l2*np.cos(theta2))

        # find all combinations of theta2, theta3 values
        theta = np.stack((theta1, theta2), axis=0)
        theta3 = phi - np.sum(theta, axis=0, keepdims=True)
        theta = np.concatenate((theta, theta3), axis=0)

        return theta

    @staticmethod
    def joint_pos_3r(link_lengths, joint_angles):
        """
        Geometry-based joint position computation for 3R chain

        Args:
            link_lengths: 3R link lengths
            joint_angles: 3R chain joint angles

        Returns:
            list of joint coordinates in the 3R plane
            [p1, p2, p3, p_end], p1 is at the origin

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
        TODO: depreciated, not used anymore

        Computes all combinations of values in two arrays

        Args:
            a: first array, shape (n,)
            b: second array, shape (n,)

        Returns:
            (2, n) matrix, value combinations on columns

        """
        return np.array(np.meshgrid(a, b)).reshape((2, len(a)*len(b)))
