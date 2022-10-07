import sys
from typing import Union
import pigpio
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

    def set(self, val: Union[str, float]):
        if isinstance(val, str):
            if val in self.positions.keys():
                pulsewidth = self.positions[val]
            else:
                print(f"{val} is not a valid claw value", file=sys.stderr)
                return
        else:
            pulsewidth = val

        self.rpi.set_servo_pulsewidth(self.pin_no, pulsewidth)


if __name__ == '__main__':
    import time

    # simple claw servo test
    claw = ClawController()

    for i in range(10):
        claw.set('open')
        time.sleep(0.5)
        claw.set('closed')
        time.sleep(0.5)
