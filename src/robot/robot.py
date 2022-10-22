import time
import numpy as np
from vision.aruco_reader import ArucoReader
from robot.motion_controller import MotionController
from robot.definitions import *
import rospy
from std_msgs.msg import String
from claw.definitions import NODE_DESIRED_CLAW_POS
from claw.claw_controller import ClawController



class Robot:
    def __init__(self):
        self.motion_controller = MotionController()
        self.aruco_reader = ArucoReader()

        # claw
        self.claw = None

        # last detected color
        self.color = None

        # claw controller publisher
        self.claw_pub = rospy.Publisher(NODE_DESIRED_CLAW_POS, String)
        # color subscriber
        self.color_sub = rospy.Subscriber(NODE_COLOR, String, callback=self._color_handler)


    def set_claw(self, claw_mode):
        self.claw_pub.publish(String(claw_mode))

    def task1(self):
        #self.aruco_reader.reset()
        self.set_claw('open')

        # wait for a non-moving cube
        cube = None
        target_pos = None
        while cube is None:
            target_position = self.motion_controller.last_position
            cube = self.aruco_reader.get_closest(target_position, verbose=True)

            if cube is not None and not cube.moving:
                target_pos = cube.avg_pos()
            else:
                if cube is not None and cube.moving:
                    print("cube moving")

        print('target_pos ', target_pos)
        # check target_pos is valid
        if not target_pos is None and not np.any(np.isnan(target_pos)):
            target_pos[-1] = FOLLOW_HEIGHT
            self.motion_controller.move_to_pos(target_pos, ts=1)
        else:
            print("got here :(")
            # delete cube from tracked cubes
            self.aruco_reader.remove_cube(cube.id)
            return False

        if cube.moving:
            print("cube started moving...")
            # delete cube from tracked cubes
            self.aruco_reader.remove_cube(cube.id)
            return False

        # ensure claw is first open
        self.set_claw('open')
        time.sleep(0.05)

        # move to grabbing position
        target_pos[-1] = GRAB_HEIGHT
        self.motion_controller.move_to_pos(target_pos, ts=1)
        time.sleep(0.05)

        # simple feedback loop for correct claw placement
        displacement = 42069
        while np.linalg.norm(displacement) > 10:
            self.motion_controller.move_to_pos(target_pos, ts=0.5)
            displacement = self.motion_controller.last_position - target_pos
            print(f"target_pos: {target_pos}, actual_pos: {self.motion_controller.last_position}")

        # attempt to grab
        self.set_claw('grip')
        time.sleep(1.0)

        # check color
        self.motion_controller.move_to_pos(COLOR_CHECK_POS, ts=1)
        time.sleep(1.0)

        color = self.aruco_reader.identify_color()
        print(f"picked up {color} block!")
        if color in COLOR_ZONES.keys():
            dump_pos = COLOR_ZONES[color]
        else:
            # return if no correct color detected, i.e. likely cube not
            # successfully grabbed
            self.motion_controller.move_to_pos(START_POS, ts=1)
            # or drop block just in case it was grabbed
            self.set_claw('open')
            # remove cube to re-detect its position on next iteration
            # TODO: not needed anymore?
            self.aruco_reader.remove_cube(cube.id)
            return False

        # move to dump zone
        self.motion_controller.move_to_pos(dump_pos, ts=1)

        # yeet cube
        self.set_claw('open')
        time.sleep(0.05)

        # move robot to suitable height
        dump_pos[-1] = FOLLOW_HEIGHT
        self.motion_controller.move_to_pos(dump_pos, ts=0.5)

        # delete cube from tracked cubes
        self.aruco_reader.remove_cube(cube.id)


    def _color_handler(self, msg: String):
        self.color = msg.data


