#!/usr/bin/env python3

import rospy
import numpy as np
from joint_controller.definitions import *
from geometry_msgs.msg import Pose, Point, Quaternion
from fiducial_msgs.msg import Fiducial, FiducialTransformArray
from cubes.py import Cube

class RobotVison():

    def __init__(self)
            """
            Constructor

            Args:
                base_height: height from ground of first joint
                link_lengths: lengths of links between joints
            """
            self.fiducial_sub = rospy.Subscriber(
                '/fiducial_transforms', # Topic name
                FiducialTransformArray, # Message type
                fiducials_callback # Callback function (required)
            self.fiducial_cubes = dict() #array of cubes

    def fiducials_callback(self):
        if len(fid.transforms) == 0:
            print('No aruco tags detected.')
            return
        elif len(fid.transforms) == 1:
            fid.transforms[0].fiducial_id 

        scale = 450/2.42
        point = np.array([fid.transforms[0].transform.translation.x, fid.transforms[0].transform.translation.y, fid.transforms[0].transform.translation.z])
        point = scale*point
        point = np.append(point, 1)
        new_p = transform_matrix@point
        if fid!= None:
            rospy.loginfo(f'x:  {fid.transforms[0].transform.translation.x*scale}')
            rospy.loginfo(f'y:  {fid.transforms[0].transform.translation.y*scale}')
            rospy.loginfo(f'z:  {fid.transforms[0].transform.translation.z*scale}')
        print('x: ', new_p[0], 'y: ', new_p[1], 'z: ', new_p[2], "\n")
        new_p = new_p[:-1]