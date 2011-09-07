#!/usr/bin/env python
"""
Module defining several outputs for the object recognition pipeline
""" 

import ecto
import ecto_geometry_msgs
import ecto_ros
from ecto_object_recognition import ros

PoseArrayPub = ecto_geometry_msgs.Publisher_PoseArray

########################################################################################################################

class Publisher(ecto.BlackBox):
    """
    Class publishing the different results of object recognition as ROS topics
    http://ecto.willowgarage.com/releases/amoeba-beta3/ros/geometry_msgs.html#Publisher_PoseArray
    """
    def __init__(self, plasm, topic_name, latched = False):
        ecto.BlackBox.__init__(self, plasm)

        self._pose_array_assembler = ros.PoseArrayAssembler()
        self._pose_pub = PoseArrayPub(topic_name=topic_name, latched = latched)

    def expose_inputs(self):
        return {'object_ids':self._pose_array_assembler['object_ids'],
                'Rs':self._pose_array_assembler['Rs'],
                'Ts':self._pose_array_assembler['Ts'],
                'image_message':self._pose_array_assembler['image_message']}

    def expose_outputs(self):
        return {}

    def expose_parameters(self):
        return {}

    def connections(self):
        return [self._pose_array_assembler['pose_message'] >> self._pose_pub[:]]

########################################################################################################################

class TabletopPublisher(ecto.BlackBox):
    """
    Class publishing the different results of object recognition as ROS topics
    """
    def __init__(self, plasm, topic_name, latched = False):
        ecto.BlackBox.__init__(self, plasm)

        self._pose_array_assembler = ros.PoseArrayAssembler()
        self._pose_pub = PoseArrayPub(topic_name=topic_name, latched = latched)

    def expose_inputs(self):
        return {'object_ids':self._pose_array_assembler['object_ids'],
                'Rs':self._pose_array_assembler['Rs'],
                'Ts':self._pose_array_assembler['Ts'],
                'image_message':self._pose_array_assembler['image_message']}

    def expose_outputs(self):
        return {}

    def expose_parameters(self):
        return {}

    def connections(self):
        return [pose_array_assembler['pose_message'] >> self._pose_pub[:]]
