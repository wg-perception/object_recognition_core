"""
Module defining several outputs for the object recognition pipeline
"""

from object_recognition_core.db.interface import ObjectDbParameters
from object_recognition_core.io.io_ros import PoseArrayAssembler, Publisher_MarkerArray
from object_recognition_core.io.sink import Sink
import ecto
import ecto_ros.ecto_geometry_msgs as ecto_geometry_msgs
import ecto_ros.ecto_std_msgs as ecto_std_msgs

PoseArrayPub = ecto_geometry_msgs.Publisher_PoseArray
MarkerArrayPub = Publisher_MarkerArray
StringPub = ecto_std_msgs.Publisher_String

########################################################################################################################

class PublisherBlackBox(ecto.BlackBox):
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
        p.declare('db_params', 'The DB parameters', ObjectDbParameters({}))

    def declare_io(self, _p, i, _o):
        i.forward_all('_pose_array_assembler')

    def configure(self, p, _i, _o):
        self._pose_array_assembler = PublisherBlackBox._pose_array_assembler()
        self._pose_pub = PublisherBlackBox._pose_pub(topic_name=p.pose_topic, latched=p.latched)
        self._object_ids_pub = PublisherBlackBox._object_ids_pub(topic_name=p.object_ids_topic, latched=p.latched)
        self._marker_pub = PublisherBlackBox._marker_pub(topic_name=p.markers_topic,latched=p.latched)
    def connections(self):
        return [self._pose_array_assembler['pose_message'] >> self._pose_pub[:],
                self._pose_array_assembler['object_ids_message'] >> self._object_ids_pub[:],
                self._pose_array_assembler['marker_message']>> self._marker_pub[:]
               ]

########################################################################################################################

class Publisher(Sink):

    @classmethod
    def type_name(cls):
        return 'publisher'

    @classmethod
    def sink(self, *args, **kwargs):
        return PublisherBlackBox(*args, **kwargs)
