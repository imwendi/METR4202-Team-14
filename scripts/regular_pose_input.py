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
from geometry_msgs.msg import Pose, Point, Quaternion

rospy.init_node('pose_input')
pose_pub = rospy.Publisher(NODE_DESIRED_POS, Pose, queue_size=10)


def input_handler(user_in: str):
    data = user_in.split(' ')

    print(data)

    if len(data) != 3:
        print("expected 'x y z", file=sys.stderr)
        return

    data = [float(val) for val in data]
    position = np.array(data)

    msg = Pose(Point(*position), Quaternion(0, 0, 0, 0))

    pose_pub.publish(msg)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        user_in = input("Input (x y z orientation): ")
        input_handler(user_in)
        rospy.sleep(0.1)
