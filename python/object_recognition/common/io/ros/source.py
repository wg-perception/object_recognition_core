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
    """Subscribes to an openni device through ROS.
    """
    _camera_info = ecto_ros.CameraInfo2Cv
    _rgb_image = ecto_ros.Image2Mat
    _depth_map = ecto_ros.Image2Mat

    def declare_params(self, p):
        pass

    def declare_io(self, _p, _i, o):
        o.forward('image', cell_name='_rgb_image',cell_key='image',doc='The RGB image from a OpenNI device.')
        o.forward('depth', cell_name='_depth_map',cell_key='image',doc='The depth map from a OpenNI device. This is a CV_32FC1, with values in meters.')
        o.forward('K', cell_name='_camera_info',cell_key='K',doc='The camera intrinsics matrix.')
        
        #this is the private synchronization subscriber setup.
        # NOTE that these are all ROS remappable on the command line in typical ros fashion
        #TODO Should these just be simple names where remapping is expected?
        subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
                    depth=ImageSub(topic_name='/camera/depth_registered/image', queue_size=0),
                    depth_info=CameraInfoSub(topic_name='/camera/depth_registered/camera_info', queue_size=0),
                    image_info=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
                 )
        #Creating this in declare io, so that i can declare the output with a concrete type.
        self._sync = ecto_ros.Synchronizer('Synchronizator', subs=subs)
        #notice that this is not a forward declare
        #its a declaration with the name, and a pointer to a tendril.
        o.declare('image_message', self._sync.outputs.at('image'))

    def configure(self, p, _i, _o):
        #these are ros message type converters.
        self._camera_info = KinectReader._camera_info('camera_info -> cv::Mat')
        self._rgb_image = KinectReader._rgb_image(swap_rgb = True)
        self._depth_map = KinectReader._depth_map()

    def connections(self):
        graph = [self._sync["image"] >> self._rgb_image["image"],
                  self._sync["depth"] >> self._depth_map["image"],
                  self._sync["image_info"] >> self._camera_info['camera_info']
                  ]
        return graph

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

