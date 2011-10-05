#!/usr/bin/env python
from object_recognition.capture.arbotix import *
import sys
MY_SERVO = 0xE9
a = ArbotiX("/dev/ttyUSB0",baud=1e6) #1 meg for e
a.enableWheelMode(MY_SERVO)
speed = 32
if len(sys.argv) > 1:
  speed = int(sys.argv[1])
a.setSpeed(MY_SERVO,speed)

