"""
Module defining several outputs for the object recognition pipeline
"""

from object_recognition_core.boost.interface import ObjectDbParameters
from object_recognition_core.ecto_cells.io_ros import MsgAssembler, VisualizationMsgAssembler, Publisher_MarkerArray
from object_recognition_core.io.sink import Sink
import ecto
import ecto_ros.ecto_geometry_msgs as ecto_geometry_msgs
import ecto_ros.ecto_std_msgs as ecto_std_msgs
from object_recognition_msgs.ecto_cells.ecto_object_recognition_msgs import Publisher_RecognizedObjectArray

PoseArrayPub = ecto_geometry_msgs.Publisher_PoseArray
MarkerArrayPub = Publisher_MarkerArray
StringPub = ecto_std_msgs.Publisher_String

########################################################################################################################

class PublisherBlackBox(ecto.BlackBox):
    """Class publishing the different results of object recognition as ROS topics
    http://ecto.willowgarage.com/releases/amoeba-beta3/ros/geometry_msgs.html#Publisher_PoseArray
    """
    _pose_pub = PoseArrayPub
    _marker_pub = MarkerArrayPub
    _object_ids_pub = StringPub
    passthrough = ecto.PassthroughN
    _recognized_object_array = Publisher_RecognizedObjectArray

    def __init__(self, do_visualize, **kwargs):
        self._do_visualize = do_visualize
        ecto.BlackBox.__init__(self, **kwargs)

    def declare_params(self, p):
        p.declare('markers_topic', 'The ROS topic to use for the marker array.', 'markers')
        p.declare('pose_topic', 'The ROS topic to use for the pose array.', 'poses')
        p.declare('object_ids_topic', 'The ROS topic to use for the object meta info string', 'object_ids')
        p.declare('recognized_object_array_topic', 'The ROS topic to use for the recognized object', 'recognized_object_array')
        p.declare('latched', 'Determines if the topics will be latched.', True)
        p.declare('db_params', 'The DB parameters', ObjectDbParameters({}))

    def declare_io(self, _p, i, _o):
        self._msg_assembler = MsgAssembler()

        i.forward_all('_msg_assembler')

    def configure(self, p, _i, _o):
        self._recognized_object_array = Publisher_RecognizedObjectArray(topic_name=p.recognized_object_array_topic, latched=p.latched)
        if self._do_visualize:
            self._visualization_msg_assembler = VisualizationMsgAssembler()
            self._pose_pub = PublisherBlackBox._pose_pub(topic_name=p.pose_topic, latched=p.latched)
            self._object_ids_pub = PublisherBlackBox._object_ids_pub(topic_name=p.object_ids_topic, latched=p.latched)
            self._marker_pub = PublisherBlackBox._marker_pub(topic_name=p.markers_topic, latched=p.latched)

    def connections(self):
        # connect to a publishing cell
        connections = [ self._msg_assembler['msg'] >> self._recognized_object_array['input']]

        if self._do_visualize:
            connections += [ self._msg_assembler['msg'] >> self._visualization_msg_assembler['msg'] ]

            connections += [self._visualization_msg_assembler['pose_message'] >> self._pose_pub[:],
                self._visualization_msg_assembler['object_ids_message'] >> self._object_ids_pub[:],
                self._visualization_msg_assembler['marker_message'] >> self._marker_pub[:]
               ]

        return connections

########################################################################################################################

class Publisher(Sink):
    """
    The publisher publishes the default ROS message.
    visualize: if that parameter is set to True, also send marker messages
    """

    @classmethod
    def config_doc(cls):
        return  """
                    # The ROS topic to use for the marker array
                    markers_topic: 'markers'
                    # The ROS topic to use for the pose array
                    pose_topic: 'poses'
                    # The ROS topic to use for the object meta info string
                    object_ids_topic: 'object_ids'
                    # The ROS topic to use for the recognized object
                    recognized_object_array_topic: 'recognized_object_array'
                    # Determines if the topics will be latched
                    latched: True
                    # The DB parameters
                    db_params: ''
                """

    @classmethod
    def type_name(cls):
        return 'publisher'

    @classmethod
    def sink(self, *args, **kwargs):
        do_visualize = kwargs.pop('visualize', False)
        return PublisherBlackBox(do_visualize, **kwargs)
