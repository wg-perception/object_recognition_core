"""
Module defining several inputs for the object recognition pipeline

All source cells will have the following outputs:
 - K [cv::Mat]
    The camera intrinsics matrix, as a 3x3 double cv::Mat
 - depth [cv::Mat]
    The rescaled depth image.
 - image [cv::Mat]
    A cv::Mat copy.
 - points3d [cv::Mat]
    The 3d points, height by width (or 1 by n_points if mask) with 3 channels (x, y and z)
"""

from abc import ABCMeta
from ecto_image_pipeline.io.source import create_source

########################################################################################################################

class Source(object):
    '''
    A Sink abstract base class
    '''

    __metaclass__ = ABCMeta

    @classmethod #see http://docs.python.org/library/abc.html#abc.ABCMeta.__subclasshook__
    def __subclasshook__(cls, C):
        if C is Source:
            #all pipelines must have atleast this function.
            if any("type_name" in B.__dict__ for B in C.__mro__):
                return True
        return NotImplemented

    @classmethod
    def config_doc_default(cls):
        '''
        Return the default documentation for the config file of that Source
        '''
        return """
               type: '%s'
               module: '%s'
               """ % (cls.type_name(), cls.__module__)

    @classmethod
    def config_doc(cls):
        '''
        Return the documentation for the config file of that Source
        It should return a string that is interpretable as YAML. It should not contain anything that is standard
        (like the 'module', the name and so on). Anyway, if you use the standard CMake test, it will fail if you do.
        The string should contain the necessary keys. For the values, put anything you want.
        '''
        raise NotImplementedError("The Source class must return a YAML string for the configuration docs.")

    @classmethod
    def type_name(cls):
        '''
        Return the code name for your source
        '''
        raise NotImplementedError("The Source %s must return a string name." % str(cls))

    @classmethod
    def source(cls, *args, **kwargs):
        '''
        Returns the source
        '''
        raise NotImplementedError("The source has to be implemented.")

    @classmethod
    def validate(cls, detector):
        """
        This ensures that the given cell exhibits the minimal interface to be
        considered a source for object recognition
        """
        outputs = dir(cell.outputs)
        #all sources must produce the following
        for x in ('K', 'image', 'depth', 'points3d'):
            if x not in outputs:
                raise NotImplementedError('This cell does not correctly implement the source interface. Must have an output named %s' % x)
        #type checks
        for x in ('K', 'image', 'depth', 'points3d'):
            type_name = cell.outputs.at(x).type_name
            #TODO add more explicit types.
            if type_name != 'cv::Mat':
                raise NotImplementedError('This cell does not correctly implement the source interface.\n'
                                          'Must have an output named %s, with type %s\n'
                                          'This cells output at %s has type %s' % (x, 'cv::Mat', x, type_name))
        return cell

########################################################################################################################

class OpenNI(Source):

    @classmethod
    def type_name(cls):
        return 'openni'

    @classmethod
    def config_doc(cls):
        from ecto_openni import FpsMode, ResolutionMode, StreamMode
        return  """
                    # The number of frames per second for the RGB image: %s
                    image_fps: ''
                    # The number of frames per second for the depth image: %s
                    depth_fps: ''
                    # The resolution for the RGB image: %s
                    image_mode: ''
                    # The resolution for the depth image: %s
                    depth_mode: ''
                    # The stream mode: %s
                    stream_mode: ''
                """ % (str(FpsMode.values.values()), str(FpsMode.values.values()),
                       str(ResolutionMode.values.values()), str(ResolutionMode.values.values()),
                       str(StreamMode.values.values()))

    @classmethod
    def source(self, *args, **kwargs):
        from ecto_openni import FpsMode, ResolutionMode, StreamMode
        for key, _val_type_name, val_type in [ ('image_fps', 'FpsMode', FpsMode), ('depth_fps', 'FpsMode', FpsMode),
                                             ('image_mode', 'ResolutionMode', ResolutionMode),
                                             ('depth_mode', 'ResolutionMode', ResolutionMode),
                                             ('stream_mode', 'StreamMode', StreamMode) ]:
            val = kwargs.get(key, None)
            if isinstance(val, str):
                for enum in val_type.values.values():
                    if val == str(enum):
                        kwargs[key] = enum
        return create_source(*('image_pipeline', 'OpenNISource'), **kwargs)
