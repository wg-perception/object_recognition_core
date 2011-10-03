#/usr/bin/env python

from arbotix import *

MY_SERVO = 0xE9

a = ArbotiX("/dev/ttyUSB0",baud=1e6)
a.disableTorque(MY_SERVO)
#print a.getPosition(MY_SERVO)
#a.setPosition(MY_SERVO,2048) # center the servo (EX-106 has 0-4095 range over ~ 280 degrees)

# set speed for rotation in wheel mode
#a.setWheelSpeed(MY_SERVO, 0, 512) # 0/1 = direction, 512 = half speed


