import sys

import rospy
import numpy as np
import time

# message types
from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from team14.msg import Pose4, IKFeedback

# local modules
from kinematics.kinematics import RobotKinematics
from joint_controller.definitions import *
from joint_controller.utils import numpify, quintic_timescale
from kinematics.utils import rot


class JointController():
    def __init__(self, base_height=None, link_lengths=None):
        # configure RobotKinematics computation class
        self.base_height = BASE_HEIGHT if base_height is None else base_height
        self.link_lengths = LINK_LENGTHS if link_lengths is None else link_lengths
        self.rk = RobotKinematics(self.base_height, self.link_lengths)

        # current joint state
        self.joint_state = JointState()

        # ____________________________ Publishers ______________________________
        # desired joint states publisher
        self.joint_pub = \
            rospy.Publisher(NODE_DESIRED_JOINT_STATES, JointState, queue_size=10)
        
        # Current pose publisher
        self.pose_pub = \
                rospy.Publisher(NODE_CURRENT_POSE, Pose4, queue_size=10)

        # IK feedback publisher
        self.ik_feedback_pub = \
                rospy.Publisher(NODE_IK_FEEDBACK, IKFeedback, queue_size=10)

        # ____________________________ Subscribers _____________________________
        # joint states subscriber
        self.joint_sub = rospy.Subscriber(NODE_JOINT_STATES,
                                          JointState,
                                          self._joint_sub_handler)

        # desired pose subscriber
        self.pose_sub =\
            rospy.Subscriber(NODE_DESIRED_POS,
                             Pose,
                             self._pose_sub_handler,
                             queue_size=10)

        # desired pose array (x, y, z, 3R orientation) subscriber
        self.pose_array_sub = rospy.Subscriber(NODE_DESIRED_POSE4,
                                               Pose4,
                                               self._pose4_sub_handler,
                                               queue_size=10)

    def move_to(self, desired_joint_angles: np.array, ts):
        """
        Move to desired joint angles via quintic time scaled trajectory

        Args:
            desired_joint_angles: desired joint angles
            ts: desired elapsed time

        """
        poly_coeffs = quintic_timescale(ts)
        initial_joint_angles = self.get_joint_angles()

        start_time = time.time()
        current_time = start_time
        while current_time < start_time + ts:

            joint_angles =\
                initial_joint_angles +\
                (desired_joint_angles - initial_joint_angles)\
                * np.polyval(poly_coeffs, current_time - start_time)
            self.publish_joint_angles(joint_angles)

            time.sleep(ts/100)
            current_time = time.time()

    def publish_joint_angles(self, joint_angles, velocity=None, verbose=True):
        """
        Publishes list of joint angles via self.joint_pub

        Args:
            joint_angles: array of joint angles to publish [rad]
            velocity: array of per joint angular velocities [rad/s]

        """
        joint_angles *= DIRECTIONAL_MULTIPLIERS

        if verbose:
            print("moving to joint angles (rad)", np.around(joint_angles, 3))
            print("moving to joint angles (deg)", np.around(np.rad2deg(joint_angles), 3))
            print()

        if velocity is None:
            velocity = np.ones(4) * DEFAULT_VELOCITY

        print('velocity ', velocity)

        # create JointState message from angles
        msg = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
            position=joint_angles,
            velocity=velocity
        )
        self.joint_pub.publish(msg)

    def get_joint_angles(self):
        """
        Returns:
            current joint angles

        """
        return np.array(np.flip(self.joint_state.position)) * DIRECTIONAL_MULTIPLIERS

    # __________________________ SUBSCRIBER HANDLERS ___________________________
    def _joint_sub_handler(self, joint_state: JointState):
        """
        Joint state callback which converts joint angles to robot position
        and end effector orientation. Corresponding Pose4 message is published.

        Args:
            joint_state: raw dynamixel joint angles

        """
        self.joint_state = joint_state
        joint_angles = self.get_joint_angles()
        joint_pos = self.rk.joint_pos(joint_angles)
        current_pose = joint_pos[:,-1]
        phi = np.sum(joint_pos[1:])
        pose_msg = current_pose.tolist()

        self.pose_pub.publish(pose_msg, phi)

    def publish_ik_feedback(self, position: np.array, reachable: bool):
        msg = IKFeedback(position, reachable)
        self.ik_feedback_pub.publish(msg)

    def _pose_sub_handler(self, pose: Pose):
        desired_position = numpify(pose.position)

        print('desired position: ', desired_position)

        joint_angles, orientation = self.rk.pick_pose_from_position(desired_position)

        if joint_angles is None:
            self.publish_ik_feedback(desired_position, False)
            return

        print('chosen orientation %.3f deg' % np.rad2deg(orientation))

        print('chosen joint angles ', joint_angles)

        # create and publish JointState message
        self.publish_joint_angles(joint_angles)

        self.publish_ik_feedback(desired_position, True)

    def _pose4_sub_handler(self, msg: Pose4):
        print('msg.position ', msg.position)
        position = np.array(msg.position)
        print(f"moving to position {position}, orientation {np.deg2rad(msg.orientation)} deg\n")
        orientation = msg.orientation

        # compute ik solutions
        ik_solution = self.rk.ik(position, orientation)

        # filter out invalid solutions
        ik_solution = self.rk.filter_ik_solution(ik_solution)

        if ik_solution is None:
            print("No valid IK solutions for this config!", file=sys.stderr)
            return

        # select optimal solution
        # TODO: in future, this will be replaced by object collision avoidance things
        joint_angles = self.rk.pick_highest_joint_2(ik_solution)

        # TODO: don't need this check anymore?
        if np.isnan(joint_angles).any():
            print("ignored NaN angles", file=sys.stderr)
            return

        # TODO: change to auto select joint angles
        self.publish_joint_angles(joint_angles, velocity=np.ones(4)*0.25)
