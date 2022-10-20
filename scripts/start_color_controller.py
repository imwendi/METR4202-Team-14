from vision.color_controller import ColorNode, CAMERA_ID

if __name__ == '__main__':
    rospy.init_node('color_node')
    colornode = ColorNode(CAMERA_ID)
    rospy.spin()