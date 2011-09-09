#/usr/bin/env python
from arbotix import *
import math
import time

MY_SERVO = 0xE9
a = ArbotiX("/dev/ttyUSB0",baud=1e6) #1 meg for e
a.enableWheelMode(MY_SERVO)
a.setSpeed(MY_SERVO,35)
while True:
    time.sleep(0.01)

a.disableTorque(MY_SERVO)



