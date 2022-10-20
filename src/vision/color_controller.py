import rospy
from math import pi, dist
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from cv_bridge import CvBridge, CvBridgeError
import webcolors


CAMERA_ID = '31708951'


class ColorNode:
    """
    Class for a node handling camera color inputs from ximea_color_detect.
    Modified from example_camera.py given in
    https://github.com/UQ-METR4202/metr4202_ximea_ros

    """
    def __init__(self, serial):
        self.bridge = CvBridge()
        self.serial = serial
        self.image_sub = rospy.Subscriber(f"/ximea_ros/ximea_{self.serial}/image_raw", Image, self._img_handler)
        self.color_pub = rospy.Publisher("/test_color", ColorRGBA, queue_size=10)
        self.color_sub = rospy.Subscriber(f"/detected_color", String, self._color_handler)

    def _img_handler(self, data):
        global img
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
        color = ColorRGBA()
        color.r = bgr[2]
        color.g = bgr[1]
        color.b = bgr[0]
        print('rgb_colors: ', color)
        self.color_pub.publish(color)

    def _color_handler(self, data):
        print('subscribed color: ', data)
