import ecto_sensor_msgs

from .BaseSource import BaseSource
import ecto_ros
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo
########################################################################################################################

class BagReader(BaseSource):
    """Subscribes to an openni device through ROS.
    """
    def declare_params(self, p):
        BaseSource.declare_params(self, p)
        p.declare('bag', "The bag file name.", "data.bag")

    def declare_io(self, p, i, o):
        BaseSource.declare_io(self, p, i, o)
        #this is the private synchronization subscriber setup.
        #NOTE that these are all ROS remappable on the command line in typical ros fashion
        #TODO Should these just be simple names where remapping is expected?
        baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                       depth=ImageBagger(topic_name='/camera/depth/image'),
                       image_info=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
                       depth_info=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
                       )
        #Creating this in declare io, so that i can declare the output with a concrete type.
        self._source = ecto_ros.BagReader('Bag Reader', bag=p.bag, baggers=baggers)
        #notice that this is not a forward declare
        #its a declaration with the name, and a pointer to a tendril.
        o.declare('image_message', self._source.outputs.at('image'))
        o.declare('depth_message', self._source.outputs.at('depth'))
        o.declare('image_info_message', self._source.outputs.at('image_info'))
        o.declare('depth_info_message', self._source.outputs.at('depth_info'))