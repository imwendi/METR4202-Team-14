#!/usr/bin/env python3

import rospy
import numpy as np
from joint_controller.definitions import *
from geometry_msgs.msg import Pose, Point, Quaternion

class Cube():
    """
    Class for saving detcted aruco cubes

    """
    def __init__(self,
                id: int,
                curr_pos: np.array,
                curr_rot: np.array:
        """
        Constructor

        """
        self.id = id
        self.curr_pos = curr_pos
        self.curr_rot = curr_rot
        self.prev_pos = None
        #self.prev_rot = None

    def is_moving(self):
        if self.prev_pos == None:
            #??
            return True

        pos_err = 0.5 
        rot_err = 0.5 

        # Check if position has changed
        if (abs(curr_pos[0] - prev_pos[0]) < pos_err) and \
           (abs(curr_pos[1] - prev_pos[1]) < pos_err) and \
           (abs(curr_pos[2] - prev_pos[2]) < pos_err):
           return False
        else:
            return True