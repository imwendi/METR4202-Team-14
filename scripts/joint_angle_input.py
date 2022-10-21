#!/usr/bin/python3

"""
launches sends stdin pose inputs to active pose_test

Give stdin inputs as x y z orientation

"""
try:
    import readline
except:
    pass

import sys
import time
import rospy
from joint_controller.definitions import *
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

rospy.init_node('joint_angle_input')
joint_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=10)


def input_handler(user_in: str):
    data = user_in.split(' ')

    print(data)

    if len(data) != 4:
        print("expected theta1 theta2 theta3 theta4", file=sys.stderr)
        return

    joint_angles = np.deg2rad(np.array([float(val) for val in data]))
    joint_angles *= np.array([1, 1, 1, -1])

    print("moving to joint angles (rad)", np.around(joint_angles, 3))
    print("moving to joint angles (deg)", np.around(np.rad2deg(joint_angles), 3))
    print()

    velocity = np.ones(4) * 1.25

    # create JointState message from angles
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
        position=joint_angles,
        velocity=velocity
    )
    joint_pub.publish(msg)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        user_in = input("Input (theta1 theta2 theta3 theta4): ")
        input_handler(user_in)
        rospy.sleep(0.1)