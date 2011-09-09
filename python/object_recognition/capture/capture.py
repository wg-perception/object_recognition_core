#!/usr/bin/env python
import ecto

from arbotix import *
import math
import time
from ecto_opencv import imgproc, calib, highgui

def kinect_highres(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.SXGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=15, depth_fps=30,
                   device_number=device_n,
                   registration=True,
                   synchronize=False,
                   device=Device.KINECT
                   )

def kinect_vga(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.VGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=30, depth_fps=30,
                   device_number=device_n,
                   registration=True,
                   synchronize=False,
                   device=Device.KINECT
                   )
def xtion_highres(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.SXGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=30, depth_fps=30,
                   device_number=device_n,
                   registration=True,
                   synchronize=True,
                   device=Device.ASUS_XTION_PRO_LIVE
                   )

def xtion_vga(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.VGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=30, depth_fps=30,
                   device_number=device_n,
                   registration=True,
                   synchronize=True,
                   device=Device.ASUS_XTION_PRO_LIVE
                   )
def find_diff(prev, current):
  max_val = 0xFFFF #16bit max_val
  min_val = 0x0
  if current - prev >= 0:
    return current - prev
  else:
    print "current", current, "prev",prev
    return (max_val - prev) + current

def rotate(a,MY_SERVO,degrees,speed):
  a.enableWheelMode(MY_SERVO)
  prev = pos = a.getPosition(MY_SERVO)
  ticks_per_degree = 16
  total_ticks = ticks_per_degree * degrees
  # set speed for rotation in joint mode
  a.setSpeed(MY_SERVO, speed)    # half speed, values between 0 and 1023
  diff = 0
  while diff < total_ticks:
      pos = a.getPosition(MY_SERVO)
      diff = find_diff(prev, pos)
      time.sleep(0.001)
  a.setSpeed(MY_SERVO, 0)
  print 'diff',float(diff)/ticks_per_degree


class TurnTable(ecto.Module):
    """ A python module that does not much."""
    def __init__(self, *args, **kwargs):
        ecto.Module.__init__(self, **kwargs)
        self.MY_SERVO = 0xE9
        self.a = ArbotiX("/dev/ttyUSB0",baud=1e6) #1 meg for e
        self.a.disableWheelMode(self.MY_SERVO,resolution=12)
        pos = self.a.getPosition(self.MY_SERVO)
        print 'initial position', pos
        assert pos != -1
        self.a.setSpeed(self.MY_SERVO,100)
        self.a.setPosition(self.MY_SERVO,0)
        while self.a.getPosition(self.MY_SERVO) > 5:
            time.sleep(0.1)
        print "At position: ", self.a.setPosition(self.MY_SERVO,0)
    @staticmethod
    def declare_params(params):
        params.declare("text", "a param.","hello there")

    @staticmethod
    def declare_io(params, inputs, outputs):
        outputs.declare("trigger", "Capture", True)

    def configure(self,params):
        self.text = params.text

    def process(self,inputs, outputs):
        rotate(self.a, self.MY_SERVO, 5, 25)
        
device = 0
capture = ecto.If('capture',cell=xtion_highres(device))
#capture = xtion_vga(device)

verter = highgui.NiConverter('verter')
fps = highgui.FPSDrawer('fps')
saver = highgui.ImageSaver("saver", filename_format='image_%05d.jpg',
                                   start=0)

table = TurnTable()

plasm = ecto.Plasm()
plasm.connect(table['trigger'] >> capture['__test__'],
              capture[:] >> verter[:],
              verter['image'] >> fps[:],
              fps[:] >> highgui.imshow('image display', name='image', waitKey=10)[:],
              verter['depth'] >> highgui.imshow('depth display', name='depth', waitKey= -1)[:],
              #fps[:] >> saver['image']
              )

if __name__ == '__main__':
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
