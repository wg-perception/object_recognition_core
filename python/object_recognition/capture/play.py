#/usr/bin/env python

from arbotix import *
import math
import time

def find_diff(prev, current):
  max_val = 0xFFFF #16bit max_val
  min_val = 0x0
  if current - prev > 0:
    return current - prev
  else:
    return (max_val - prev) + current

def rotate(a,MY_SERVO,degrees,degree_delta,speed):
  a.enableWheelMode(MY_SERVO)
  prev = a.getPosition(MY_SERVO)
  total = 0
  ticks_per_degree = 16
  full_circle = ticks_per_degree * degrees
  while total < full_circle:
      # set speed for rotation in joint mode
      a.setSpeed(MY_SERVO, speed)    # half speed, values between 0 and 1023
      pos = a.getPosition(MY_SERVO)
      diff = find_diff(prev, pos)
      while diff < ticks_per_degree * degree_delta:
          pos = a.getPosition(MY_SERVO)
          diff = find_diff(prev, pos)
          time.sleep(0.001)
      prev = pos
      a.setSpeed(MY_SERVO, 0)
      time.sleep(0.5)
      total += diff
      print 'total',float(total)/ticks_per_degree




MY_SERVO = 0xE9
a = ArbotiX("/dev/ttyUSB0",baud=1e6) #1 meg for e
a.disableWheelMode(MY_SERVO,resolution=12)
pos = a.getPosition(MY_SERVO)
print pos
assert pos != -1
a.setSpeed(MY_SERVO,100)
a.setPosition(MY_SERVO,0)
while a.getPosition(MY_SERVO) > 5:
    time.sleep(0.1)
print "At position: ", a.setPosition(MY_SERVO,0) # center the servo (EX-106 has 0-4095 range over ~ 280 degrees)

rotate(a, MY_SERVO, 360, 5, 25)

a.disableTorque(MY_SERVO)



