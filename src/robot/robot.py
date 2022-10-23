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

        # if last detected timetable state was moving
        self.turntable_moving = False
        # last detection timetable stop time
        self.last_timetable_stop = 0


    def set_claw(self, claw_mode):
        self.claw_pub.publish(String(claw_mode))

    def task1(self):
        self.wait_for_turntable()
        if self.task1_old():
            self.sort_cube()



    def sort_cube(self):
        """
        Assuming a cube is grapped, checks its colour and then places it down

        """
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
            return False

        # move to dump zone
        self.motion_controller.move_to_pos(dump_pos, ts=1)

        # yeet cube
        self.set_claw('open')
        time.sleep(0.1)
        # self.set_claw('grip')
        # time.sleep(0.5)

        # move robot to suitable height
        dump_pos[-1] = FOLLOW_HEIGHT
        self.motion_controller.move_to_pos(dump_pos, ts=0.5)

        return True

    def wait_for_turntable(self):
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
                #print("%.3fs left to grab cube" % (GRAB_CUBE_DURATION - time_past_last_stop))
                return True


    def task1_old(self):
        #self.aruco_reader.reset()
        self.set_claw('open')

        # wait for a non-moving cube
        cube = None
        target_pos = None
        while cube is None:
            target_position = self.motion_controller.last_position
            cube = self.aruco_reader.get_closest(target_position, verbose=True)

            if cube is not None:
                target_pos = cube.avg_pos()

            # if cube is not None and not cube.moving:
            #     target_pos = cube.avg_pos()
            # else:
            #     if cube is not None and cube.moving:
            #         #print("cube moving")
            #         pass

        #print('target_pos ', target_pos)
        # check target_pos is valid
        if not target_pos is None and not np.any(np.isnan(target_pos)):
            target_pos[-1] = FOLLOW_HEIGHT
            self.motion_controller.move_to_pos(target_pos, ts=1)
        else:
            #print("got here :(")
            # delete cube from tracked cubes
            #self.aruco_reader.remove_cube(cube.id)
            return False

        if self.aruco_reader.turntable_moving():
            print("cube started moving...")
            # delete cube from tracked cubes
            #self.aruco_reader.remove_cube(cube.id)
            return False

        # ensure claw is first open
        self.set_claw('open')

        # TODO: don't need this?
        #time.sleep(0.2)

        # move to grabbing position
        target_pos[-1] = GRAB_HEIGHT
        self.motion_controller.move_to_pos(target_pos, ts=0.2)
        time.sleep(0.1)

        # simple feedback loop for correct claw placement
        # displacement = 42069
        # while np.linalg.norm(displacement) > 10:
        #     self.motion_controller.move_to_pos(target_pos, ts=0.5)
        #     displacement = self.motion_controller.last_position - target_pos
        #     print(f"target_pos: {target_pos}, actual_pos: {self.motion_controller.last_position}")


        self.set_claw('grip')
        # TODO: don't need this?
        #time.sleep(0.5)

        # check color
        #self.set_claw('grip')
        #self.aruco_reader.remove_cube(cube.id)

        return True



    def _color_handler(self, msg: String):
        self.color = msg.data


