"""
Simple node for controlling the SG90 servo for the claw

"""

import sys
from typing import Union
import pigpio
import rospy
from std_msgs.msg import String
from claw.definitions import *


class ClawController():
    def __init__(self,
                 pin_no=CLAW_PIN,
                 closed_pos=CLOSE,
                 grip_pos=GRIP,
                 open_pos=OPEN):
        """
        Constructor

        Args:
            pin_no: servo pin number
            closed_pos: closed position PWM value
            grip_pos: grip position PWM value
            open_pos: open position PWM value
        """

        # reconfigure PWM pin
        self.rpi = pigpio.pi()
        self.pin_no = pin_no
        self.rpi.set_mode(self.pin_no, pigpio.INPUT)
        time.sleep(0.5)
        self.rpi.set_mode(self.pin_no, pigpio.OUTPUT)
        self.rpi.set_servo_pulsewidth(self.pin_no, 0)
        time.sleep(0.5)

        # set positions
        self.positions =\
            {
            'close': closed_pos,
            'grip': grip_pos,
            'open': open_pos
            }

        # configure ROS subscriber
        self.claw_sub = rospy.Subscriber(NODE_DESIRED_CLAW_POS,
                                         data_class=String,
                                         callback=self._claw_handler,
                                         queue_size=10)

    def set(self, val: Union[str, float]):
        """
        Set servo to given position

        Args:
            val: PWM value to set or 'closed', 'grip' or 'open'

        Returns:

        """

        isopen = False
        if isinstance(val, str):
            val = val.lower()   # convert to lower case
            if val in self.positions.keys():
                pulsewidth = self.positions[val]
                if val.lower() == 'open':
                    isopen = True
            else:
                print(f"{val} is not a valid claw value", file=sys.stderr)
                return
        else:
            pulsewidth = val

        print(f"set pulsewidth {pulsewidth}")
        self.rpi.set_servo_pulsewidth(self.pin_no, pulsewidth)

        # disable pwn in open position, i.e. when servo is unused
        if isopen:
            time.sleep(0.1)
            self.rpi.set_servo_pulsewidth(self.pin_no, 0)

    def _claw_handler(self, String):
        """
        Subscriber handler for setting claw position

        Args:
            String: claw mode to set,  'closed', 'grip' or 'open'

        """
        pos = String.data
        self.set(pos)


if __name__ == '__main__':
    import time

    # simple claw servo test
    claw = ClawController()

    for i in range(10):
        claw.set('open')
        time.sleep(0.5)
        claw.set('closed')
        time.sleep(0.5)
