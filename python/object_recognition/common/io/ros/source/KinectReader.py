"""
Module defining several inputs for the object recognition pipeline
"""
import ecto
import ecto_ros
import ecto_sensor_msgs
from .BaseSource import BaseSource

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

########################################################################################################################
class KinectReader(BaseSource):
    """Subscribes to an openni device through ROS.
    """
    def declare_io(self, p, i, o):
        BaseSource.declare_io(self, p, i, o)
        #this is the private synchronization subscriber setup.
        #NOTE that these are all ROS remappable on the command line in typical ros fashion
        #TODO Should these just be simple names where remapping is expected?
        subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
                    depth=ImageSub(topic_name='/camera/depth_registered/image', queue_size=0),
                    depth_info=CameraInfoSub(topic_name='/camera/depth_registered/camera_info', queue_size=0),
                    image_info=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
                 )
        #Creating this in declare io, so that i can declare the output with a concrete type.
        self._source = ecto_ros.Synchronizer('Synchronizator', subs=subs)
        #notice that this is not a forward declare
        #its a declaration with the name, and a pointer to a tendril.
        o.declare('image_message', self._source.outputs.at('image'))

