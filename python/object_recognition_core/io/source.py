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
    def type_name(cls):
        '''
        Return the code name for your source
        '''
        raise NotImplementedError("The Source class must return a string name.")

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
    def source(self, *args, **kwargs):
        return create_source(*('image_pipeline', 'OpenNISource'), **kwargs)
