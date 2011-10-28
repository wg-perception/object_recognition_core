'''
Loaders for all object recognition pipelines
'''
from abc import ABCMeta, abstractmethod

class DetectionPipeline:
    ''' An abstract base class for creating object detection pipelines.
    '''
    __metaclass__ = ABCMeta

    @abstractmethod
    def create_pipeline(self, argv=None):
        '''
        Concrete implementations of Detection pipelines should use this as the most general creation point.
        
        :param argv: Command line arguments, if argv is None, then it is expected that sys.argv will be used.
                     These command line args should be taken into consideration when constructing the pipeline.

        :returns: An ecto.Plasm instance, which contains the pipeline
                  that when executed for 1 iteration will produce a detection result.  This may change...
        '''
        pass

    @classmethod #see http://docs.python.org/library/abc.html#abc.ABCMeta.__subclasshook__
    def __subclasshook__(cls, C):
        if cls is DetectionPipeline:
            #all pipelines must have atleast this function.
            if any("create_pipeline" in B.__dict__ for B in C.__mro__):
                return True
        return NotImplemented

    @classmethod
    def create(cls, argv=[]):
        from .tod import TODDetection
        td = TODDetection()
        return td.create_pipeline(argv)