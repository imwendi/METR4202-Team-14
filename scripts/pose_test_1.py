import rospy
from joint_controller.joint_controller import JointController

rospy.init_node('pose_test_1')
joint_controller = JointController()

if __name__ == '__main__':
    rospy.spin()

