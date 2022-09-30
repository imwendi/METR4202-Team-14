import rospy

# message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# local modules
from kinematics.kinematics_base import KinematicsBase
from joint_controller.definitions import *
from joint_controller.utils import numpify


class JointController():
    def __init__(self, link_lengths=None):
        # current joint state
        self.joint_state = JointState()
        # joint states subscriber
        self.joint_state_sub = rospy.Subscriber(NODE_JOINT_STATES,
                                                JointState,
                                                self._update_joint_state)

        # desired pose subscriber
        self.desired_pose_sub =\
            rospy.Subscriber(NODE_DESIRED_POS, Pose, self._publish_desired_pose)

        # desired joint states publisher
        self.desired_joint_state_pub =\
            rospy.Publisher(NODE_DESIRED_JOINT_STATES, JointState, queue_size=10)

        # configure RobotKinematics computation class
        if link_lengths is None:
            # use default link lengths
            link_lengths = DEFAULT_LINK_LENGTHS
        self.link_lengths = link_lengths
        self.rk = KinematicsBase(link_lengths)

    def _publish_desired_pose(self, pose: Pose):
        current_joint_angles = self._get_joint_angles()
        desired_position = numpify(pose.position)
        phi = 0 # TODO: for testing only!

        print("desired pos is ", desired_position)

        new_joint_angles =\
            self.rk.ik_closest_norm(desired_position, phi, current_joint_angles)

        print("moving to ", new_joint_angles)

        # create and publish JointState message
        msg = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
            position=new_joint_angles,
            velocity=np.ones(4)*DEFAULT_VELOCITY
        )
        self.desired_joint_state_pub.publish(msg)

    def _get_joint_angles(self):
        """
        Returns:
            current joint angles

        """
        return np.array(self.joint_state.position)

    def _update_joint_state(self, joint_state: JointState):
        self.joint_state = joint_state
