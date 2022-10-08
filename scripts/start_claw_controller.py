#!/usr/bin/python3

import rospy
from claw.claw_controller import ClawController

rospy.init_node('claw_controller')
claw_controller = ClawController()

if __name__ == '__main__':
    rospy.spin()
