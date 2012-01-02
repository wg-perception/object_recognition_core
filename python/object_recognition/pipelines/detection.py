'''
ABC for Detection pipelines
'''
from abc import ABCMeta

class DetectionPipeline:
    ''' An abstract base class for creating object training pipelines.
    '''
    __metaclass__ = ABCMeta

    @classmethod
    def type_name(cls):
        '''
        Return the code name for your pipeline. eg. 'TOD', 'LINEMOD', 'mesh', etc...
        '''
        raise NotImplementedError("The detection pipeline class must return a string name.")

    @classmethod #see http://docs.python.org/library/abc.html#abc.ABCMeta.__subclasshook__
    def __subclasshook__(cls, C):
        if C is DetectionPipeline:
            #all pipelines must have atleast this function.
            if any("type_name" in B.__dict__ for B in C.__mro__):
                return True
        return NotImplemented

    @classmethod
    def detector(cls, *args, **kwargs):
        '''
        Returns the detector cell which needs to have inputs compatible with the used Source's, and one of the outputs
        has to be a std::vector<object_recognition::common::PoseResult>
        '''
        raise NotImplementedError("The detector has to be implemented.")

    @classmethod
    def validate(cls, detector):
        """
        Makes sure that the detector is valid
        """
        # check for the pose_results output
        if 'pose_results' not in detector.outputs:
            raise NotImplementedError("The detector needs to have a 'pose_results' output tendril.")
        # check for the right type for the pose_results output
        type_name = detector.outputs.at('pose_results').type_name
        if type_name not in ['std::vector<object_recognition::io::PoseResult, std::allocator<object_recognition::io::PoseResult> >']:
            raise NotImplementedError("The detector does not have a 'pose_results' tendril of the right type.\n"
                                      "Must have an output named 'pose_results', with type %s\n"
                                      "This cells output at 'pose_results' has type %s" % ('std::vector<PoseResult>', type_name))
