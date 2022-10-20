#!/usr/bin/env python3

import rospy
import numpy as np
from joint_controller.definitions import *
from geometry_msgs.msg import Pose, Point, Quaternion
from claw.claw_controller import ClawController
from kinematics.kinematics_base import KinematicsBase
from kinematics.utils import rot



from fiducial_msgs.msg import Fiducial
from fiducial_msgs.msg import FiducialTransformArray
from sensor_msgs.msg import JointState
from team14.msg import Pose4


class ArucoFollow():

    def __init__(self):

        self.pose_pub = rospy.Publisher(NODE_DESIRED_POS, Pose, queue_size=10)
        
        self.sub = rospy.Subscriber( '/fiducial_transforms', # Topic name
                                FiducialTransformArray, # Message type
                                self.print_fiducials # Callback function (required)
        )
        self.pose_sub = rospy.Subscriber('current_pose',
                                    Pose4,
                                    self._pose_sub_handler)

        self.claw = ClawController()
        self.transform_matrix = np.array([[0, 1, 0, 190],
                                        [1, 0, 0, 0],
                                        [0, 0, -1, 426+25],
                                        [0, 0, 0, 1]])
        self.current_pos = np.zeros(3)


    def print_fiducials(self, fid: FiducialTransformArray):
        if len(fid.transforms) > 0:
            scale = 450/2.42
            point = np.array([fid.transforms[0].transform.translation.x, fid.transforms[0].transform.translation.y, fid.transforms[0].transform.translation.z])
            point = scale*point
            point = np.append(point, 1)
            new_p = self.transform_matrix@point
            # if fid!= None:
            #     rospy.loginfo(f'x:  {fid.transforms[0].transform.translation.x*scale}')
            #     rospy.loginfo(f'y:  {fid.transforms[0].transform.translation.y*scale}')
            #     rospy.loginfo(f'z:  {fid.transforms[0].transform.translation.z*scale}')
            print('x: ', new_p[0], 'y: ', new_p[1], 'z: ', new_p[2], "\n")
            new_p = new_p[:-1]

            self.claw.set('open')
            msg = Pose(Point(*new_p), Quaternion(0, 0, 0, 0))
            self.pose_pub.publish(msg)
            print('current_pos: ', self.current_pos)
            #print('new_p: ', new_p)
            print('shape: ',self.current_pos.shape)

            print(np.linalg.norm(self.current_pos - new_p))
            if np.linalg.norm(self.current_pos - new_p) < 10:

                self.claw.set('grip')


        #rospy.loginfo(f'fid:  {fid.transforms[0].transform}')
    def _pose_sub_handler(self, T: Pose4):
        self.current_pos = np.array(T.position)
        #self.current_pos = rot(0, 0, np.pi)@np.array(T.position)
        #print(self.current_pos)



def main():

    af = ArucoFollow()

    # Initialise node with any node name
    rospy.init_node('aruco_follow_node')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()
