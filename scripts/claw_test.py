#!/usr/bin/python3

import time
from claw.claw_controller import ClawController

# simple claw servo test
claw = ClawController()

for i in range(10):
    time.sleep(0.5)
    claw.set('open')
    time.sleep(0.5)
    claw.set('grip')
    time.sleep(0.5)
    claw.set('close')
    time.sleep(0.5)
