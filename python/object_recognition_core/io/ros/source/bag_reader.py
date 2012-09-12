"""
Module defining several outputs for the object recognition pipeline
"""

from object_recognition_core.io.source import Source
from ecto_image_pipeline.io.source import create_source

########################################################################################################################

class BagReader(Source):

    @classmethod
    def config_doc(cls):
        return  """
                    # The path of the bag to read
                    bag: ''
                """

    @classmethod
    def type_name(cls):
        return 'bag_reader'

    @classmethod
    def source(self, *args, **kwargs):
        return create_source(*('image_pipeline', 'BagReader'), **kwargs)
