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

        # configure PWM pin
        self.rpi = pigpio.pi()
        self.pin_no = pin_no
        self.pin = self.rpi.set_mode(self.pin_no, pigpio.OUTPUT)

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
        if isinstance(val, str):
            val = val.lower()   # convert to lower case
            if val in self.positions.keys():
                pulsewidth = self.positions[val]
            else:
                print(f"{val} is not a valid claw value", file=sys.stderr)
                return
        else:
            pulsewidth = val

        self.rpi.set_servo_pulsewidth(self.pin_no, pulsewidth)

    def _claw_handler(self, String):
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
