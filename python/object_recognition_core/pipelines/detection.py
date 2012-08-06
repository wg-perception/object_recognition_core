'''
ABC for Detection pipelines
'''
from abc import ABCMeta
import ecto
from object_recognition_core.utils.json_helper import dict_to_cpp_json_str
import warnings

class DetectionBlackbox(ecto.BlackBox):
    """
    This blackbox contains a detection pipeline and a cell that simply forwards its parameters
    """
    def __init__(self, detection_pipeline, *args, **kwargs):
        self._detector = detection_pipeline.detector(*args, **kwargs)
        self._parameters = kwargs
        self._info = ecto.Constant(value=dict_to_cpp_json_str(kwargs))
        ecto.BlackBox.__init__(self)

    def declare_io(self, _p, i, o):
        i.forward_all('_detector')
        o.forward_all('_detector')
        o.forward('parameters', cell_name = '_info', cell_key = 'out')

    #def configure(self, p, _i, _o):
    #    self._detector_cell = self._detector
    def connections(self):
        return []

class DetectionPipeline:
    '''
    An abstract base class for creating object training pipelines.
    When creating a new detection pipeline, it has to inherit from this one and implement the
    type_name and detector functions.
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

def validate_detection_pipeline(detector):
    """
    Makes sure that the detector is valid
    """
    # check for the pose_results output
    if 'pose_results' not in detector.outputs:
        warnings.warn("The detector needs to have a 'pose_results' output tendril.")
    # check for the right type for the pose_results output
    type_name = detector.outputs.at('pose_results').type_name
    if type_name not in ['std::vector<object_recognition_core::common::PoseResult, std::allocator<object_recognition_core::common::PoseResult> >']:
        warnings.warn("The detector does not have a 'pose_results' tendril of the right type.\n"
                                  "Must have an output named 'pose_results', with type %s\n"
                                  "This cells output at 'pose_results' has type %s" % ('std::vector<PoseResult>', type_name))
