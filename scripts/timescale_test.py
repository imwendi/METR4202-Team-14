#!/usr/bin/python3

try:
    import readline
except:
    pass

import sys
import rospy
import numpy as np
from joint_controller.joint_controller import JointController

rospy.init_node('timescale_test')
joint_controller = JointController()


def input_handler(user_in: str):
    global last_claw_mode
    data = user_in.split(' ')

    if not (len(data) == 5):
        print("expected angle1 angle2 angle3 angle4 ts", file=sys.stderr)
        return

    data = [float(val) for val in data]
    joint_angles = np.deg2rad(np.array(data[:-1]))
    ts = data[-1]

    joint_controller.move_to(joint_angles, ts)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        user_in = input("Input angle1 angle2 angle3 angle4 ts: ")
        input_handler(user_in)
        rospy.sleep(0.1)
