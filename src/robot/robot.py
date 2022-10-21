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
        self.claw = ClawController()

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
        self.claw.set('open')
        #time.sleep(2)

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
            self.motion_controller.move_to_pos(target_pos)
        else:
            print("got here :(")
            return False

        if cube.moving:
            print("cube started moving...")
            return False

        # ensure claw is first open
        self.claw.set('open')
        time.sleep(0.05)

        # move to grabbing position
        target_pos[-1] = GRAB_HEIGHT
        self.motion_controller.move_to_pos(target_pos)
        time.sleep(0.05)

        # simple feedback loop for correct claw placement
        displacement = 42069
        while np.linalg.norm(displacement) > 10:
            self.motion_controller.move_to_pos(target_pos)
            displacement = self.motion_controller.last_position - target_pos
            print(f"target_pos: {target_pos}, actual_pos: {self.motion_controller.last_position}")

        # attempt to grab
        self.claw.set('grip')
        #time.sleep(0.5)
        # #time.sleep(1.5)
        # if not self.try_grip():
        #     print("grip failed :(")
        #     return False

        # check color
        self.motion_controller.move_to_pos(COLOR_CHECK_POS)
        # sleep to update color
        time.sleep(1.5)

        color = self.aruco_reader.identify_color()
        print(f"picked up {color} block!")
        if color in COLOR_ZONES.keys():
            dump_pos = COLOR_ZONES[color]
        else:
            # return if no correct color detected, i.e. likely cube not
            # successfully grabbed
            self.motion_controller.move_to_pos(START_POS)
            # or drop block just in case it was grabbed
            self.set_claw('open')
            #time.sleep(1)

            # remove cube to re-detect its position on next iteration
            self.aruco_reader.remove_cube(cube.id)
            return False

        # move to dump zone
        self.motion_controller.move_to_pos(dump_pos)

        # yeet cube
        self.claw.set('open')
        time.sleep(0.05)

        # move robot to suitable height
        dump_pos[-1] = FOLLOW_HEIGHT
        self.motion_controller.move_to_pos(dump_pos)

        # delete cube from tracked cubes
        self.aruco_reader.remove_cube(cube.id)

    def try_grip(self):
        successful_positioning = False
        y_deltas = [0, -10, -20, 10, 20]
        last_position = self.motion_controller.last_position

        for y_delta in y_deltas:
            # move to above block with y_delta
            desired_pos = last_position + np.array([0, 1, 0]) * y_delta
            desired_pos[-1] = FOLLOW_HEIGHT
            self.motion_controller.move_to_pos(desired_pos)

            # attempt to move down
            desired_pos[-1] = GRAB_HEIGHT
            self.motion_controller.move_to_pos(desired_pos)
            time.sleep(0.1)

            # check if height is correct
            print(self.motion_controller.last_position[-1], GRAB_HEIGHT)
            if np.abs(self.motion_controller.last_position[-1] - GRAB_HEIGHT) < 5:
                successful_positioning = True
                print("successful position!")
                break
            else:
                # reset to position above desired pos
                desired_pos[-1] = GRAB_HEIGHT + 10
                self.motion_controller.move_to_pos(desired_pos)

            time.sleep(0.5)

        if successful_positioning:
            # attempt to grab
            self.claw.set('grip')
            time.sleep(0.5)

        return successful_positioning


    def _color_handler(self, msg: String):
        self.color = msg.data


