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
from geometry_msgs.msg import Point, Quaternion
from team14.msg import Position

rospy.init_node('pose_input')
pose_pub = rospy.Publisher(NODE_TIME_SCALED_POS, Position, queue_size=10)
claw_pub = rospy.Publisher(NODE_DESIRED_CLAW_POS, String, queue_size=10)

last_claw_mode = 'open'

def input_handler(user_in: str):
    global last_claw_mode
    data = user_in.split(' ')

    if not (4 <= len(data) <= 5):
        print("expected x y z ts (claw mode)", file=sys.stderr)
        return

    claw_mode = last_claw_mode
    if len(data) == 5:
        claw_mode = str(data[-1])
    last_claw_mode = claw_mode

    data = [float(val) for val in data]
    position = np.array(data)[:3]
    ts = data[-1]

    msg = Position(position, ts)
    pose_pub.publish(msg)
    claw_pub.publish(claw_mode)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        user_in = input("Input x y z ts (claw mode): ")
        input_handler(user_in)
        rospy.sleep(0.1)
