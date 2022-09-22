#!/usr/bin/env python3

"""
test which moves arm up and done vertically a little

"""
import time
import rospy
from geometry_msgs.msg import Pose, Point
from joint_controller.definitions import *

STEP_SIZE = 2   # [mm]
START_Z = L     # [mm]
END_Z = L - 20      # [mm]
DELAY = 0.5       # [s]

pose_pub = rospy.Publisher(NODE_DESIRED_POS,
                           Pose,
                           queue_size=1)

rospy.init_node('vertical_test')


if __name__ == '__main__':
    step = STEP_SIZE
    current_z = START_Z

    while not rospy.is_shutdown():
        current_z = max(END_Z, min(START_Z, current_z + step))

        if current_z in [START_Z, END_Z]:
            step *= -1

        msg = Pose(position=Point(0, 0, current_z))
        pose_pub.publish(msg)

        print('current z', current_z)

        time.sleep(DELAY)