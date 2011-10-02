"""
Module defining several inputs for the object recognition pipeline
""" 
import ecto
import ecto_ros
import ecto_sensor_msgs

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

########################################################################################################################

class BaseSource(ecto.BlackBox):
    """A source that uses ROS to produce image, depth, and camera info.
    """
    _camera_info = ecto_ros.CameraInfo2Cv
    _rgb_image = ecto_ros.Image2Mat
    _depth_map = ecto_ros.Image2Mat
    _source = None #this should be allocated in by implementers

    def declare_params(self, p):
        pass

    def declare_io(self, _p, _i, o):
        o.forward('image', cell_name='_rgb_image',cell_key='image',doc='The RGB image from a OpenNI device.')
        o.forward('depth', cell_name='_depth_map',cell_key='image',doc='The depth map from a OpenNI device. This is a CV_32FC1, with values in meters.')
        o.forward('K', cell_name='_camera_info',cell_key='K',doc='The camera intrinsics matrix.')

    def configure(self, p, _i, _o):
        #these are ros message type converters.
        self._camera_info = BaseSource._camera_info('camera_info -> cv::Mat')
        self._rgb_image = BaseSource._rgb_image(swap_rgb = True)
        self._depth_map = BaseSource._depth_map()

    def connections(self):
        graph = [self._source["image"] >> self._rgb_image["image"],
                  self._source["depth"] >> self._depth_map["image"],
                  self._source["image_info"] >> self._camera_info['camera_info']
                  ]
        return graph