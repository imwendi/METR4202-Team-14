#!/usr/bin/env python3

import rospy
import numpy as np
from joint_controller.definitions import *
from geometry_msgs.msg import Pose, Point, Quaternion

pose_pub = rospy.Publisher(NODE_DESIRED_POS, Pose, queue_size=10)
transform_matrix = np.array([[0, 1, 0, 190],
                            [1, 0, 0, 0],
                            [0, 0, -1, 400],
                            [0, 0, 0, 1]])

from fiducial_msgs.msg import Fiducial
from fiducial_msgs.msg import FiducialTransformArray

def print_fiducials(fid: FiducialTransformArray):
    if len(fid.transforms) > 0:
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

        msg = Pose(Point(*new_p), Quaternion(0, 0, 0, 0))
        pose_pub.publish(msg)
        #rospy.loginfo(f'fid:  {fid.transforms[0].transform}')

def main():

    # Create subscriber
    sub = rospy.Subscriber(
        '/fiducial_transforms', # Topic name
        FiducialTransformArray, # Message type
        print_fiducials # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('aruco_follow_node')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()