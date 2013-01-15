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

import ecto
from ecto_image_pipeline.io.source import create_source

########################################################################################################################

class Source(object):
    """
    This is a base class for a source: you don't need to have your source cell inherit from that class but if you do,
    it will make sure that its inputs/outputs fit the ORK standard (which is good if you want to interact with
    the official ORK pipelines).
    You need to call the BlackBox constructor in your __init__ first and then this function. Typically, your __init__ is
    class Foo(ecto.BlackBox, Source):
        def __init__(self, *args, **kwargs):
            ecto.BlackBox.__init__(self, *args, **kwargs)
            Source.__init__(self)
    """

    def __init__(self):
        """
        This ensures that the given cell exhibits the minimal interface to be
        considered a source for object recognition
        """
        outputs = dir(self.outputs)
        # all sources must produce the following
        for x in ('K', 'image', 'depth'):
            if x not in outputs:
                raise NotImplementedError('This cell with doc\n%s\nodes not correctly implement the source interface. '
                                          'Must have an output named %s' % (self.__doc__, x))
        # type checks
        for x in ('K', 'image', 'depth'):
            type_name = self.outputs.at(x).type_name
            # TODO add more explicit types.
            if type_name != 'cv::Mat':
                raise NotImplementedError('The cell with doc\n%s\n does not correctly implement the source interface.\n' % 
                                           self.__doc__ + 'Must have an output named %s, with type %s\n' % (x, 'cv::Mat') + 
                                           'This cells output at %s has type %s' % (x, type_name))

########################################################################################################################

class OpenNI(ecto.BlackBox, Source):
    """
    A source for any ORK pipeline, interface with a Kinect/ASUS Xtion pro using the openni driver
    """
    def __init__(self, *args, **kwargs):
        from ecto_openni import FpsMode, ResolutionMode, StreamMode
        for key, val_type in [ ('image_fps', FpsMode), ('depth_fps', FpsMode), ('image_mode', ResolutionMode),
                                             ('depth_mode', ResolutionMode), ('stream_mode', StreamMode) ]:
            val = kwargs.get(key, None)
            if isinstance(val, str):
                for enum in val_type.values.values():
                    if val == str(enum):
                        kwargs[key] = enum
        ecto.BlackBox.__init__(self, *args, **kwargs)
        Source.__init__(self)

    def declare_cells(self, p):
        return {'main': create_source(*('image_pipeline', 'OpenNISource'), **p)}

    def declare_params(self, p):
        from ecto_openni import FpsMode, ResolutionMode, StreamMode
        p.declare('image_fps', "The number of frames per second for the RGB image: %s" % str(FpsMode.values.values()))
        p.declare('depth_fps', "The number of frames per second for the depth image: %s" % str(FpsMode.values.values()))
        p.declare('image_mode', "The resolution for the RGB image: %s" % str(ResolutionMode.values.values()))
        p.declare('depth_mode', "The resolution for the depth image: %s" % str(ResolutionMode.values.values()))
        p.declare('stream_mode', "The stream mode: %s" % str(StreamMode.values.values()))

    def declare_forwards(self, _p):
        return ({}, {'main': 'all'}, {'main': 'all'})

    def connections(self, _p):
        return [self.main]
