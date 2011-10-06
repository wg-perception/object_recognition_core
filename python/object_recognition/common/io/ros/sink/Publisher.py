"""
Module defining several outputs for the object recognition pipeline
"""

import ecto
import ecto_geometry_msgs, ecto_std_msgs
from ecto_object_recognition.io_ros import PoseArrayAssembler, Publisher_MarkerArray

PoseArrayPub = ecto_geometry_msgs.Publisher_PoseArray
MarkerArrayPub = Publisher_MarkerArray
StringPub = ecto_std_msgs.Publisher_String
########################################################################################################################

class Publisher(ecto.BlackBox):
    """Class publishing the different results of object recognition as ROS topics
    http://ecto.willowgarage.com/releases/amoeba-beta3/ros/geometry_msgs.html#Publisher_PoseArray
    """
    _pose_array_assembler = PoseArrayAssembler
    _pose_pub = PoseArrayPub
    _marker_pub = MarkerArrayPub
    _object_ids_pub = StringPub

    def declare_params(self, p):
        p.declare('markers_topic', 'The ROS topic to use for the marker array.', 'markers')
        p.declare('pose_topic', 'The ROS topic to use for the pose array.', 'poses')
        p.declare('object_ids_topic', 'The ROS topic to use for the object meta info string', 'object_ids')
        p.declare('latched', 'Determines if the topics will be latched.', True)
        p.declare('mapping', 'A mapping from object id to mesh id', {'na':'na'})

    def declare_io(self, _p, i, _o):
        i.forward_all('_pose_array_assembler')

    def configure(self, p, _i, _o):
        self._pose_array_assembler = Publisher._pose_array_assembler(mapping=p.mapping)
        self._pose_pub = Publisher._pose_pub(topic_name=p.pose_topic, latched=p.latched)
        self._object_ids_pub = Publisher._object_ids_pub(topic_name=p.object_ids_topic, latched=p.latched)
        self._marker_pub = Publisher._marker_pub(topic_name=p.markers_topic,latched=p.latched)
    def connections(self):
        return [self._pose_array_assembler['pose_message'] >> self._pose_pub[:],
                self._pose_array_assembler['object_ids_message'] >> self._object_ids_pub[:],
                self._pose_array_assembler['marker_message']>> self._marker_pub[:]
               ]

