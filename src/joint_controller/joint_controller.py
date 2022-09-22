import rospy

# message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# local modules
from kinematics.kinematics import RobotKinematics
from definitions import *


class JointController():
    def __init__(self, link_lengths=None):
        # desired pose subscriber
        self.desired_pose_sub =\
            rospy.Subscriber(NODE_DESIRED_POS, Pose, self.get_pose())

        # desired joint states publisher
        self.desired_joint_state_pub =\
            rospy.Publisher(NODE_DESIRED_JOINT_STATES, Pose, queue_size=10)

        # configure RobotKinematics computation class
        if link_lengths is None:
            # use default link lengths
            link_lengths = DEFAULT_LINK_LENGTHS
        self.link_lengths = link_lengths
        self.rk = RobotKinematics(link_lengths)

    def get_pose(self):
        pass


