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
        self.claw_pub = rospy.Publisher(NODE_DESIRED_CLAW_POS, String, queue_size=10)
        # color subscriber
        self.color_sub = rospy.Subscriber(NODE_COLOR, String, callback=self._color_handler)


    def set_claw(self, claw_mode):
        self.claw_pub.publish(String(claw_mode))

    def task1(self):
        """
        Task 1 loop

        Returns:

        """
        # exit if turntable is moving or there are no cubes
        if self.aruco_reader.turntable_empty():
            print("turntable empty!")
            return -1

        # wait for a non-moving cube
        cube = None
        target_pos = None
        while cube is None:
            cube = self.aruco_reader.get_closest(self.motion_controller.last_position)

            if cube is not None:
                target_pos = cube.get_latest_data('position')
                if not np.any(np.isnan(target_pos)):
                    target_pos[-1] = FOLLOW_HEIGHT
                    self.motion_controller.move_to_pos(target_pos, ts=1)

        #
        # if self.aruco_reader.turntable_moving():
        #     print("turntable moving!")
        #     return -1

        # move to nearest block
        # if not self.go_to_nearest_cube():
        #     # TODO: remove
        #     # print("failed to go to nearest")
        #     return -1

        # attempt pickup and drop off

        # return to some initial position


    def go_to_cube(self, cube):
        """
        Attempts to move to nearest cube

        Returns: True if attempted to move robot, False is no cubes close by

        """
        # no valid cubes to move to
        if cube is None:
            # TODO: remove this!
            # print("no valid cubes to go to")
            return False

        print(f"closed cube is {cube.id}!")

        # try move to cube
        cube_position = cube.get_latest_data()[1]
        desired_position = cube_position; desired_position[-1] = FOLLOW_HEIGHT

        # print(f"moving to cube {cube.id} at {desired_position}")
        start = time.time()
        self.motion_controller.move_to_pos(desired_position)
        end = time.time()
        #print(f"Took {np.around(end-start, 3)}s to get to cube {cube.id}", flush=True)

        # check if cube position was actually reached (x, y directions)
        cube_position = cube.get_latest_data()[1]

        # if np.linalg.norm(self.motion_controller.last_position - desired_position) > CLOSENESS_THRESHOLD:
        #     print("Failed to move close to cube :(")
        #
        #     return False

        # make position micro adjustments if necessary
        # start_time = time.time()
        # while (time.time() - start_time < TIMEOUT):
        #     timestamp, cube_position, _, _ = cube.get_latest_data()
        #     desired_position = cube_position; desired_position[-1] = FOLLOW_HEIGHT
        #
        #     self.motion_controller.move_to_pos(desired_position)
        #
        #     if np.linalg.norm(current_position - desired_position) < CLOSENESS_THRESHOLD:
        #         # gotten close enough to cube
        #         return True
        #
        #     # TODO: place this earlier in loop?
        #     current_position = self.motion_controller.last_position

        return True


    def _color_handler(self, msg: String):
        self.color = msg.data


