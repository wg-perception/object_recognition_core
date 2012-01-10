#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow
from ecto_opencv.calib import DepthTo3d
from image_pipeline import Rectifier, StereoModelLoader, DepthRegister, CameraModelToCv, CV_INTER_NN
from ecto_openni import OpenNICapture, DEPTH_RGB, DEPTH_IR, RGB, IR, IRGamma, enumerate_devices
from image_pipeline_conversion import MatToPointCloudXYZRGB
from ecto_pcl import PointCloudT2PointCloud, CloudViewer, XYZRGB

openni_reg = True
print enumerate_devices()

capture = OpenNICapture('Camera',stream_mode=DEPTH_RGB, registration=openni_reg, latched=False)
depthTo3d = DepthTo3d('Depth ~> 3d')
to_xyzrgb = MatToPointCloudXYZRGB('OpenCV ~> PCL')
pcl_cloud = PointCloudT2PointCloud('conversion',format=XYZRGB)
cloud_viewer = CloudViewer('Cloud Display')


plasm = ecto.Plasm()
plasm.connect(capture['K'] >> depthTo3d['K'],
              capture['image'] >> imshow('Image Display')[:],
              capture['depth'] >> depthTo3d['depth'],
              depthTo3d['points3d'] >> to_xyzrgb['points'],
              capture['image'] >> to_xyzrgb['image'],
              to_xyzrgb[:] >> pcl_cloud[:],
              pcl_cloud[:] >> cloud_viewer[:]
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture Kinect depth and RGB and register them.', locals=vars())
