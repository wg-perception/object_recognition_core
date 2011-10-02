#!/usr/bin/env python
"""
Module defining several outputs for the object recognition pipeline
"""

import ecto
import ecto_geometry_msgs, ecto_std_msgs
from ecto_object_recognition.io_ros import PoseArrayAssembler

PoseArrayPub = ecto_geometry_msgs.Publisher_PoseArray
StringPub = ecto_std_msgs.Publisher_String
########################################################################################################################

class Publisher(ecto.BlackBox):
    """Class publishing the different results of object recognition as ROS topics
    http://ecto.willowgarage.com/releases/amoeba-beta3/ros/geometry_msgs.html#Publisher_PoseArray
    """
    _pose_array_assembler = PoseArrayAssembler
    _pose_pub = PoseArrayPub
    _object_ids_pub = StringPub

    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)

    def declare_params(self, p):
        p.declare('pose_topic', 'The ROS topic to use for the pose array.', 'poses')
        p.declare('object_ids_topic', 'The ROS topic to use for the object meta info string', 'object_ids')
        p.declare('latched', 'Determines if the topics will be latched.', True)

    def declare_io(self, _p, i, _o):
        i.forward_all('_pose_array_assembler')

    def configure(self, p, _i, _o):
        self._pose_array_assembler = Publisher._pose_array_assembler()
        self._pose_pub = Publisher._pose_pub(topic_name=p.pose_topic, latched=p.latched)
        self._object_ids_pub = Publisher._object_ids_pub(topic_name=p.object_ids_topic, latched=p.latched)
    def connections(self):
        return [self._pose_array_assembler['pose_message'] >> self._pose_pub[:],
                self._pose_array_assembler['object_ids_message'] >> self._object_ids_pub[:]]

########################################################################################################################

if __name__ == '__main__':
    p = Publisher()
    print p.__doc__