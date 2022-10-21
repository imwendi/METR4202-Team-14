import time
import numpy as np
from vision.aruco_reader import ArucoReader
from robot.motion_controller import MotionController
from robot.definitions import *
import rospy
from std_msgs.msg import String
from claw.definitions import NODE_DESIRED_CLAW_POS



class Robot:
    def __init__(self):
        self.motion_controller = MotionController()
        self.aruco_reader = ArucoReader()

        # claw controller publisher
        self.claw_pub = rospy.Publisher(NODE_DESIRED_CLAW_POS, String)


    def set_claw(self, claw_mode):
        self.claw_pub.publish(String(claw_mode))

    def task1(self):
        #time.sleep(1)
        self.set_claw('open')
        time.sleep(2)

        # wait for a non-moving cube
        cube = None
        target_pos = None
        while cube is None:
            target_position = self.motion_controller.last_position
            cube = self.aruco_reader.get_closest(target_position, verbose=True)

            if cube is not None and not cube.moving:
                target_pos = cube.avg_pos()
            else:
                print()

        print('target_pos ', target_pos)
        # check target_pos is valid
        if not np.any(np.isnan(target_pos)):
            target_pos[-1] = FOLLOW_HEIGHT
            self.motion_controller.move_to_pos(target_pos)
        else:
            print("got here :(")
            return False

        #time.sleep(1)
        target_pos[-1] = GRAB_HEIGHT
        self.motion_controller.move_to_pos(target_pos)

        #time.sleep(2)
        self.set_claw('close')

        time.sleep(1)
        self.motion_controller.move_to_pos(COLOR_CHECK_POS)

        #time.sleep(2)
        self.motion_controller.move_to_pos(DUMP_POS)

        #time.sleep(2)
        self.set_claw('open')
        time.sleep(2)

        self.aruco_reader.remove_cube(cube.id)


