''''
Module defining several outputs for the object recognition pipeline
'''

from abc import ABCMeta
from object_recognition_core.io import GuessCsvWriter

########################################################################################################################

class Sink(object):
    '''
    A Sink abstract base class
    '''

    __metaclass__ = ABCMeta

    @classmethod #see http://docs.python.org/library/abc.html#abc.ABCMeta.__subclasshook__
    def __subclasshook__(cls, C):
        if C is Sink:
            #all pipelines must have atleast this function.
            if any("type_name" in B.__dict__ for B in C.__mro__):
                return True
        return NotImplemented

    @classmethod
    def type_name(cls):
        '''
        Return the code name for your sink
        '''
        raise NotImplementedError("The Sink class must return a string name.")

    @classmethod
    def sink(cls, *args, **kwargs):
        '''
        Returns the sink
        '''
        raise NotImplementedError("The sink has to be implemented.")

    @classmethod
    def validate(cls, detector):
        """
        This ensures that the given cell exhibits the minimal interface to be
        considered a sink for object recognition
        """
        inputs = dir(cell.inputs)
        #all sources must produce the following
        for x in ['pose_results']:
            if x not in inputs:
                raise NotImplementedError('This cell does not correctly implement the sink interface. Must have an input named %s' % x)
        #type checks
        for x in ['pose_results']:
            type_name = cell.inputs.at(x).type_name
            #TODO add more explicit types.
            if type_name not in ['std::vector<object_recognition::common::PoseResult, std::allocator<object_recognition::common::PoseResult> >']:
                raise NotImplementedError('This cell does not correctly implement the sink interface.\n'
                                          'Must have an output named %s, with type %s\n'
                                          'This cells input at %s has type %s' % (x, 'std::vector<PoseResult>', x, type_name))
        return cell

########################################################################################################################

class GuessCsvWriterPython(Sink):

    @classmethod
    def type_name(cls):
        return 'csv_writer'

    @classmethod
    def sink(self, *args, **kwargs):
        return GuessCsvWriter()
