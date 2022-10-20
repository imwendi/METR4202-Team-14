import numpy as np
import time
import rospy
from joint_controller.definitions import *
from geometry_msgs.msg import Pose, Point, Quaternion
from team14.msg import IKFeedback, Pose4


class StateMachine:
    def __init__(self):
        self.last_desired_pos = np.zeros(3)

        # last ik feedback
        self.ik_feedback = None
        self.feedback_sub = rospy.Subscriber(NODE_IK_FEEDBACK, IKFeedback, callback=self._ik_feedback_handler, queue_size=10)
        # last position
        self.last_position = np.zeros(3)
        self.pose_sub = rospy.Subscriber('current_pose',
                                    Pose4,
                                    self._pose_sub_handler)

        # desired position publisher
        self.pose_pub = rospy.Publisher(NODE_DESIRED_POS, Pose, queue_size=10)

    def move_to_pos(self, position: np.array):
        msg = Pose(Point(*position), Quaternion(0, 0, 0, 0))
        self.pose_pub.publish(msg)
        start_time = time.time()

        while (time.time() - start_time) < 5: # TODO: max wait time
            feedback_pos, reachable = self.ik_feedback

            # check if position is valid from IK
            if feedback_pos == position and reachable == False:
                return False

            # check if position reached
            if np.linalg.norm(position, self.last_position) < 10: # TODO: closeness threshold
                return True

    def _ik_feedback_handler(self, feedback: IKFeedback):
        self.ik_feedback = (feedback.position, feedback.reachable)

    def _pose_sub_handler(self, T: Pose4):
        self.last_position = np.array(T.position)
