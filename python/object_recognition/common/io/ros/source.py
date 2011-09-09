#!/usr/bin/env python
"""
Module defining several inputs for the object recognition pipeline
""" 

import ecto
import ecto_ros
import ecto_sensor_msgs
from ecto_opencv import calib, highgui
import sys

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

########################################################################################################################

class KinectReader(ecto.BlackBox):
    """
    Blackbox that reasd data from the Kinect, returns an image and a camera frame point cloud
    """
    def __init__(self, plasm, debug = False):
        ecto.BlackBox.__init__(self, plasm)

        subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
                    depth=ImageSub(topic_name='/camera/depth_registered/image', queue_size=0),
                    depth_info=CameraInfoSub(topic_name='/camera/depth_registered/camera_info', queue_size=0),
                    image_info=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
                 )

        self._sync = ecto_ros.Synchronizer('Synchronizator', subs=subs)
        self._camera_info = ecto_ros.CameraInfo2Cv('camera_info -> cv::Mat')
        self._im2mat_rgb = ecto_ros.Image2Mat(swap_rgb = True)
        self._im2mat_depth = ecto_ros.Image2Mat()
        self._depth_to_3d = calib.DepthTo3d()
        self._debug = debug

    def expose_inputs(self):
        return {}

    def expose_outputs(self):
        return {'image': self._im2mat_rgb['image'],
                'points3d': self._depth_to_3d['points3d'],
                'K': self._camera_info['K'],
                'image_message': self._sync["image"]
                }

    def expose_parameters(self):
        return {}

    def connections(self):
        # not sure on where to put this but that has to be done whenever the cell is used (and not necessarily created)
        ecto_ros.init(sys.argv, "ecto_node")

        connections = [self._sync["image"] >> self._im2mat_rgb["image"],
                  self._sync["depth"] >> self._im2mat_depth["image"],
                  self._sync["image_info"] >> self._camera_info['camera_info'],
                  self._camera_info['K'] >> self._depth_to_3d['K'],
                  self._im2mat_depth['image'] >> self._depth_to_3d['depth']
                  ]
        # TODO : reset that to DEBUG
        #if self._debug:
        connections += [self._im2mat_depth[:] >> highgui.imshow(name='kinect depth',waitKey=1)[:]]
        return connections

########################################################################################################################

class BagReader(ecto.BlackBox):
    def __init__(self, plasm, baggers, bag):
        ecto.BlackBox.__init__(self, plasm)

        self._im2mat_rgb = ecto_ros.Image2Mat()
        self._camera_info_conversion = ecto_ros.CameraInfo2Cv()
        self._point_cloud_conversion = ecto_pcl_ros.Message2PointCloud(format=ecto_pcl.XYZRGB)
        self._point_cloud_conversion2 = ecto_pcl.PointCloud2PointCloudT(format=ecto_pcl.XYZRGB)
        self._bag_reader = ecto_ros.BagReader('Bag Reader',
                                baggers=baggers,
                                bag=options.bag,
                              )

    def expose_inputs(self):
        return {}

    def expose_outputs(self):
        return {'image': self._im2mat_rgb['image'],
                'point_cloud': self._point_cloud_conversion2['output']}

    def expose_parameters(self):
        return {}

    def connections(self):
        return (self._bag_reader['image'] >> self._im2mat_rgb['image'],
                  self._bag_reader['camera_info'] >> self._camera_info_conversion['camera_info'],
                  self._bag_reader['point_cloud'] >> self._point_cloud_conversion['input'],
                  self._point_cloud_conversion['output'] >> self._point_cloud_conversion2['input'])
