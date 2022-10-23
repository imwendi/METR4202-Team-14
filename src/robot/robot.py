import time
from typing import Optional

import numpy as np
from vision.aruco_reader import ArucoReader
from robot.motion_controller import MotionController
from robot.definitions import *
import rospy
from std_msgs.msg import String
from claw.definitions import NODE_DESIRED_CLAW_POS
from vision.cube import Cube
from vision.definitions import CUBE_TIMEOUT


"""
High-level robot state machine class

"""
class Robot:
    def __init__(self):
        """
        Constructor

        """
        self.motion_controller = MotionController()
        self.aruco_reader = ArucoReader()

        # claw
        self.claw = None
        # last detected color
        self.color = None
        # claw controller publisher
        self.claw_pub = rospy.Publisher(NODE_DESIRED_CLAW_POS, String, queue_size=10)
        # color subscriber
        self.color_sub = rospy.Subscriber(NODE_COLOR, String, callback=self._color_handler)

        # if last detected timetable state was moving
        self.turntable_moving = False
        # last detection timetable stop time
        self.last_timetable_stop = 0

    def task1(self, moving_turntable=True):
        """
        Single iteration of task1/2

        Args:
            moving_turntable: set True to assume the turn table will be moving

        """
        if moving_turntable:
            self.wait_for_turntable()
        grabbed_cube = self.grab_cube()
        if grabbed_cube is not None:
            self.sort_cube(grabbed_cube)

    def grab_cube(self):
        """
        Attempts to grab a cube.

        Returns:
            Cube object of grabbed cube, or None if not grabbed.

        """
        self.set_claw('open')

        # wait for a non-moving cube
        cube = None
        target_pos = None
        while cube is None:
            target_position = self.motion_controller.last_position
            cube = self.aruco_reader.get_closest(target_position, verbose=True)
            if cube is not None:
                target_pos = cube.avg_pos()

        # check target_pos is valid
        if not target_pos is None and not np.any(np.isnan(target_pos)):
            target_pos = self.adjust_follow_pos(target_pos)
            self.motion_controller.move_to_pos(target_pos, ts=1)
        else:
            print("got here :(")
            # delete cube from tracked cubes
            self.aruco_reader.remove_cube(cube.id)
            return None

        # ensure claw is first open
        self.set_claw('open')
        time.sleep(0.1)

        # check if cube has moved again
        cube_new_position = cube.avg_pos()
        if np.linalg.norm(cube_new_position[:2] - target_pos[:2]) > CLOSENESS_THRESHOLD:
            print("cube moved away :(")

            self.return_home()
            self.aruco_reader.remove_cube(cube.id)
            return None

        # move to grabbing position
        target_pos = self.adjust_grab_pos(target_pos)
        self.motion_controller.move_to_pos(target_pos, ts=0.2)
        time.sleep(0.1)

        self.set_claw('grip')

        return cube

    def sort_cube(self, cube: Optional[Cube] = None):
        """
        Assuming a cube is grabbed, checks its colour and then places it down.

        Args:
            cube: grabbed cube to sort

        """
        start_time = time.time()

        self.motion_controller.move_to_pos(COLOR_CHECK_POS, ts=0.5)
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
            return False

        # double check if cube id is still visible on table - if so then not
        # picked up properly
        if cube is not None:
            if cube.update_time - start_time > CUBE_TIMEOUT:
                self.return_home(0.5)
                return False

        # move to dump zone
        self.motion_controller.move_to_pos(dump_pos, ts=1)

        # yeet cube
        self.set_claw('open')
        time.sleep(0.1)
        self.return_home(0.5)

        return True

    def wait_for_turntable(self):
        """
        Blocks and continuously checking turntable until it has stopped with
        sufficient time for tasks to be done.

        Returns:
            True if turntable is immobile with sufficient remaining time before
            next rotation cycle.

        """
        while True:
            current_time = time.time()
            turntable_moving = self.aruco_reader.turntable_moving()

            if turntable_moving and not self.turntable_moving:
                # turntable started moving again
                print("turntable started moving!")
            elif not turntable_moving and self.turntable_moving:
                # turntable just stopped
                self.last_timetable_stop = current_time
                print("turntable just stopped!")

            self.turntable_moving = turntable_moving

            time_past_last_stop = np.abs(current_time - self.last_timetable_stop)
            if not turntable_moving and time_past_last_stop < GRAB_CUBE_DURATION:
                # enough time to try grab cube
                return True

    def set_claw(self, claw_mode):
        """
        Publish desired claw mode

        Args:
            claw_mode: desired mode to set

        """
        self.claw_pub.publish(String(claw_mode))

    def adjust_follow_pos(self, target_pos):
        """
        Adjusts follow target position if cube is relatively far from turntable center.

        Args:
            target_pos: initial target position

        Returns:
            adjusted target position

        """
        displacement = target_pos - TURNTABLE_CENTER
        x_displacement, y_displacement = displacement[:2]

        if np.abs(y_displacement) > Y_ADJUST_THRESHOLD:
            target_pos[1] += 8*np.sign(y_displacement)
            print("adjusted Y!")
        else:
            print("Y displacement was just ", y_displacement)

        target_pos[-1] = FOLLOW_HEIGHT

        return target_pos

    def adjust_grab_pos(self, target_pos):
        """
        Adjusts grab target position if cube is relatively far from turntable center.

        Args:
            target_pos: initial target position

        Returns:
            adjusted target position

        """
        displacement = target_pos - TURNTABLE_CENTER
        x_displacement = displacement[0]

        target_pos[-1] = GRAB_HEIGHT

        if np.abs(x_displacement) > Y_ADJUST_THRESHOLD:
            target_pos[-1] += 8*np.sign(x_displacement)

            target_pos[0] += 5*np.sign(x_displacement)

            print(f"adjusted X, Z by {5*np.sign(x_displacement)}!")
            print("X displacement was ", x_displacement)
        else:
            print("X displacement was just", x_displacement)

        return target_pos

    def return_home(self, ts=0.5):
        """
        Moves robot to "home" position ready for another pick up task.

        Args:
            ts: Time to take to go home

        """
        current_position = self.motion_controller.last_position
        current_y = current_position[1]
        if current_y > 0:
            target_pos = LEFT_HOME
        else:
            target_pos = RIGHT_HOME

        target_pos[-1] = FOLLOW_HEIGHT

        self.motion_controller.move_to_pos(target_pos, ts)

    def _color_handler(self, msg: String):
        """
        Callback to update about current camera detected color

        """
        self.color = msg.data


