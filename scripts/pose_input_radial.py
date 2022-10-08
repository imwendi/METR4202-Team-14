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
from claw.definitions import *
from std_msgs.msg import String
from team14.msg import Pose4

rospy.init_node('pose_input')
pose_pub = rospy.Publisher(NODE_DESIRED_POSE4, Pose4, queue_size=10)
claw_pub = rospy.Publisher(NODE_DESIRED_CLAW_POS, String, queue_size=10)


last_claw_mode = 'open'

def input_handler(user_in: str):
    global last_claw_mode

    data = user_in.split(' ')

    print(data)

    if len(data) < 4:
        print("expected 'x y z orientation <claw_mode>", file=sys.stderr)
        return

    claw_mode = last_claw_mode
    if len(data) == 5:
        claw_mode = str(data[-1])
    last_claw_mode = claw_mode

    data = [float(val) for val in data[:4]]
    r, theta, z = np.array(data[:3])
    theta = np.deg2rad(theta)
    position = np.array([r*np.cos(theta), r*np.sin(theta), z])
    orientation = np.deg2rad(data[3])

    pose_pub.publish(Pose4(position, orientation))
    claw_pub.publish(String(claw_mode))


if __name__ == '__main__':
    while not rospy.is_shutdown():
        user_in = input("Input (radius direction z orientation): ")
        input_handler(user_in)
        rospy.sleep(0.1)
