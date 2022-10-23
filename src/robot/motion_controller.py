import numpy as np
import time
import rospy
import tf2_ros

from joint_controller.definitions import *
from geometry_msgs.msg import Pose, Point, Quaternion
from team14.msg import IKFeedback, Pose4, Position

"""
Class to send and receive high level desired position messages to joint nodes
"""
class MotionController:
    def __init__(self,
                 closeness_threshold=10,
                 max_wait_time=5):
        """
        Constructor

        Args:
            closeness_threshold: threshold determining if two positions are close
            max_wait_time: maximum wait time for a robot path to finish
        """
        self.closeness_threshold = closeness_threshold
        self.max_wait_time = max_wait_time

        # last requested position
        self.last_desired_pos = np.zeros(3)

        # last position
        self.last_position = np.zeros(3)
        self.pose_sub = rospy.Subscriber(NODE_CURRENT_POSE,
                                         Pose4,
                                         self._pose_sub_handler)

        # last ik feedback
        self.ik_feedback = None
        self.feedback_sub = rospy.Subscriber(NODE_IK_FEEDBACK,
                                             IKFeedback,
                                             callback=self._ik_feedback_handler,
                                             queue_size=10)

        # desired position publisher
        self.pose_pub = rospy.Publisher(NODE_DESIRED_POS, Pose, queue_size=10)

        # desired position (with time scaling) publisher
        self.time_scaled_pos_pub = rospy.Publisher(NODE_TIME_SCALED_POS,
                                                   Position,
                                                   queue_size=10)

    def move_to_pos(self, position: np.array, ts=None):
        """
        Command robot to move to a position. Can use a quintic time scaled
        trajectory.

        Args:
            position: Desired position
            ts: Desired travel time for time scaled trajectory, else no time
                scaling is used.

        Returns:
            True if the position was successfully reached

        """
        # ignore NaN positions
        if (np.any(np.isnan(position))):
            return False
        print("position ", position)

        if ts is None:
            msg = Pose(Point(*position), Quaternion(0, 0, 0, 0))
            self.pose_pub.publish(msg)
        else:
            msg = Position(position, ts)
            self.time_scaled_pos_pub.publish(msg)

        start_time = time.time()
        while (time.time() - start_time) < self.max_wait_time:
            # check if position reached
            if np.linalg.norm(position - self.last_position) < self.closeness_threshold:
                print("finished motion!")
                return True


    def _ik_feedback_handler(self, feedback: IKFeedback):
        """
        Callback for reading and updating IK feedback messages

        """
        self.ik_feedback = (feedback.position, feedback.reachable)

    def _pose_sub_handler(self, T: Pose4):
        """
        Callback to read and update current robot pose

        """
        self.last_position = np.array(T.position)
