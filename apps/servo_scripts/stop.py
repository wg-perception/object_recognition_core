#!/usr/bin/env python
from object_recognition.capture.arbotix import *

MY_SERVO = 0xE9

a = ArbotiX("/dev/ttyUSB0",baud=1e6)
a.disableTorque(MY_SERVO)

