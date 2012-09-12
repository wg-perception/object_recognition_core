"""
Module defining several outputs for the object recognition pipeline
"""

from object_recognition_core.io.source import Source
from ecto_image_pipeline.io.source import create_source

########################################################################################################################

class RosKinect(Source):

    @classmethod
    def config_doc(cls):
        return  """
                """

    @classmethod
    def type_name(cls):
        return 'ros_kinect'

    @classmethod
    def source(self, *args, **kwargs):
        return create_source(*('image_pipeline', 'OpenNISubscriber'), **kwargs)
