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
from joint_controller.msg import Pose4

rospy.init_node('pose_input')
pose_pub = rospy.Publisher(NODE_DESIRED_POSE4, Pose4, queue_size=10)


def input_handler(user_in: str):
    data = user_in.split(' ')

    print(data)

    if len(data) != 4:
        print("expected 'x y z orientation", file=sys.stderr)
        return

    data = [float(val) for val in data]
    position = np.array(data[:-1])
    orientation = data[-1]

    pose_pub.publish(Pose4(position, orientation))


if __name__ == '__main__':
    while not rospy.is_shutdown():
        user_in = input("Input (x y z orientation): ")
        input_handler(user_in)
        rospy.sleep(0.1)
