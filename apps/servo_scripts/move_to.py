#/usr/bin/env python

from object_recognition.capture.arbotix import *
import time

MY_SERVO = 0xE9
a = ArbotiX("/dev/ttyUSB0")
a.disableWheelMode(MY_SERVO,resolution=12)
a.setSpeed(MY_SERVO,100)
a.setPosition(MY_SERVO,0)
time.sleep(1)
a.setPosition(MY_SERVO,4095)
a.disableTorque(MY_SERVO)

