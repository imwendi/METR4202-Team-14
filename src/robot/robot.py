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

        # last detected color
        self.color = None

        # claw controller publisher
        self.claw_pub = rospy.Publisher(NODE_DESIRED_CLAW_POS, String)
        # color subscriber
        self.color_sub = rospy.Subscriber(NODE_COLOR, String, callback=self._color_handler)


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
        if not target_pos is None and not np.any(np.isnan(target_pos)):
            target_pos[-1] = FOLLOW_HEIGHT
            self.motion_controller.move_to_pos(target_pos)
        else:
            print("got here :(")
            return False

        # move to grabbing position
        target_pos[-1] = GRAB_HEIGHT
        self.motion_controller.move_to_pos(target_pos)

        # attempt to grab
        self.set_claw('close')
        time.sleep(1)

        # check color
        self.motion_controller.move_to_pos(COLOR_CHECK_POS)
        # sleep to update color
        time.sleep(2)

        color = self.aruco_reader.identify_color()
        print(f"picked up {color} block!")
        if color in COLOR_ZONES.keys():
            dump_pos = COLOR_ZONES[color]
        else:
            # return if no correct color detected, i.e. likely cube not
            # successfully grabbed
            self.motion_controller.move_to_pos(HOME_POS)

            # remove cube to re-detect its position on next iteration
            self.aruco_reader.remove_cube(cube.id)
            return False

        # move to dump zone
        self.motion_controller.move_to_pos(dump_pos)

        # yeet cube
        self.set_claw('open')
        time.sleep(2)

        # delete cube from tracked cubes
        self.aruco_reader.remove_cube(cube.id)

    def _color_handler(self, msg: String):
        self.color = msg.data


