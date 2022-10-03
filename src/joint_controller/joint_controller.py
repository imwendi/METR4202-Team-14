import sys

import rospy
import numpy as np

# message types
from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from joint_controller.msg import Pose4

# local modules
from kinematics.kinematics import RobotKinematics
from joint_controller.definitions import *
from joint_controller.utils import numpify


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

        # ____________________________ Subscribers _____________________________
        # joint states subscriber
        self.joint_sub = rospy.Subscriber(NODE_JOINT_STATES,
                                          JointState,
                                          self._joint_sub_handler)

        # desired pose subscriber
        # self.pose_sub =\
        #     rospy.Subscriber(NODE_DESIRED_POS, Pose, self._pose_sub_handler)

        # desired pose array (x, y, z, 3R orientation) subscriber
        self.pose_array_sub = rospy.Subscriber(NODE_DESIRED_POSE4,
                                               Pose4,
                                               self._pose4_sub_handler)

    def publish_joint_angles(self, joint_angles, velocity=None, verbose=True):
        """
        Publishes list of joint angles via self.joint_pub

        Args:
            joint_angles: list of joint angles to publish [rad]
            velocity: array of per joint angular velocities [rad/s]

        """
        joint_angles *= DIRECTIONAL_MULTIPLIERS

        if verbose:
            print("moving to joint angles (rad)", np.around(joint_angles, 3))
            print("moving to joint angles (deg)", np.around(np.rad2deg(joint_angles), 3))
            print()

        if velocity is None:
            velocity = np.ones(4) * DEFAULT_VELOCITY

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
        return np.array(self.joint_state.position)

    # __________________________ SUBSCRIBER HANDLERS ___________________________

    def _joint_sub_handler(self, joint_state: JointState):
        self.joint_state = joint_state

    def _pose_sub_handler(self, pose: Pose):
        #TODO: depreciated?
        desired_position = numpify(pose.position)
        phi = 0 # TODO: for testing only!

        print("desired pos is ", desired_position)

        # todo: change to autoselect joint angles
        new_joint_angles =\
            self.rk.ik(desired_position, phi)[:, 3]

        # create and publish JointState message
        self.publish_joint_angles(new_joint_angles)

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
