#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Funny code
import random

# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


# Your inverse kinematics function
# This one doesn't actually do it though...
def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    # TODO: Have fun :)
    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(dummy_joint_states())


# Funny code
def dummy_joint_states() -> JointState:
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    # Funny code
    msg.position = [
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5)
    ]
    return msg


def main():
    global pub
    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber
    sub = rospy.Subscriber(
        'desired_pose', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('metr4202_w7_prac')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()