#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import imshow
from ecto_openni import OpenNICapture, DEPTH_RGB, enumerate_devices
from image_pipeline_conversion import MatToPointCloudXYZRGB
from ecto_pcl import PointCloudT2PointCloud, CloudViewer, XYZRGB
from object_recognition.common.io.standalone import OpenNISource

capture = OpenNISource()
to_xyzrgb = MatToPointCloudXYZRGB('OpenCV ~> PCL')
pcl_cloud = PointCloudT2PointCloud('conversion', format=XYZRGB)
cloud_viewer = CloudViewer('Cloud Display')

plasm = ecto.Plasm()
plasm.connect(capture['image'] >> imshow('Image Display')[:],
              capture['points3d'] >> to_xyzrgb['points'],
              capture['image'] >> to_xyzrgb['image'],
              to_xyzrgb[:] >> pcl_cloud[:],
              pcl_cloud[:] >> cloud_viewer[:]
              )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Capture Kinect depth and RGB and register them.', locals=vars())
