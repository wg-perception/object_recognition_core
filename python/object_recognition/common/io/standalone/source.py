#!/usr/bin/env python
"""
Module defining several inputs for the object recognition pipeline
"""

import ecto
from ecto_opencv import calib, highgui

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
########################################################################################################################

class KinectReader(ecto.BlackBox):
    """
    Blackbox that reasd data from the Kinect, returns an image and a camera frame point cloud
    """
    def __init__(self, plasm, camera_file, debug=False):
        ecto.BlackBox.__init__(self, plasm)
        self._openni = ecto.If('Xtion PRO Live',cell=xtion_highres(0))
        self._openni.inputs.__test__ = True
        self._camera_info = calib.CameraIntrinsics(camera_file=camera_file)
        self._converters = highgui.NiConverter(rescale=True)
        self._debug = debug

    def expose_inputs(self):
        return {}

    def expose_outputs(self):
        return {'image': self._converters['image'],
                'depth': self._converters['depth'],
                'K': self._camera_info['K'],
                'D': self._camera_info['D'],
                'image_size': self._camera_info['image_size'],
                '__test__': self._openni['__test__']
                }

    def expose_parameters(self):
        return {}

    def connections(self):
        connections = [self._openni[:] >> self._converters[:],
                  ]
        return connections

