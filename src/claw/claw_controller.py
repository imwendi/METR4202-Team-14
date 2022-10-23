import sys
import time
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

        # TODO: remove this??
        if isopen:
            time.sleep(0.1)
            self.rpi.set_servo_pulsewidth(self.pin_no, 0)

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
