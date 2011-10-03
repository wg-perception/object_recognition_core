#/usr/bin/env python

from arbotix import *
import math
import time

MY_SERVO = 0xE9
a = ArbotiX("/dev/ttyUSB0")
a.disableWheelMode(MY_SERVO,resolution=12)
a.setSpeed(MY_SERVO,100)
a.setPosition(MY_SERVO,0)
time.sleep(1)
a.setPosition(MY_SERVO,4095)
#print a.setPosition(MY_SERVO,0) # center the servo (EX-106 has 0-4095 range over ~ 280 degrees)
#time.sleep(0.1)
#rotate(45, 5, a, MY_SERVO)
a.disableTorque(MY_SERVO)

# set speed for rotation in wheel mode
#a.setWheelSpeed(MY_SERVO, 0, 512) # 0/1 = direction, 512 = half speed


