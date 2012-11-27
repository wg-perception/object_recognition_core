'''
ABC for Detection pipelines
'''
from abc import ABCMeta
from object_recognition_core.ecto_cells.io import PipelineInfo
from object_recognition_core.utils.json_helper import dict_to_cpp_json_str
import ecto
import warnings
import yaml

class DetectionBlackbox(ecto.BlackBox):
    """
    This blackbox contains a detection pipeline and a cell that simply forwards its parameters
    Each executed pipeline is actually wrapped in such an object so that outputs can be linked
    to where they are coming from (through the info)
    """
    def __init__(self, detection_pipeline, *args, **kwargs):
        self._detector = detection_pipeline.detector(*args, **kwargs)
        self._parameters = kwargs
        self._info = PipelineInfo(parameters=dict_to_cpp_json_str(kwargs))
        ecto.BlackBox.__init__(self, *args, **kwargs)
        
    def declare_params(self, p):
        p.forward_all('_detector')

    def declare_io(self, _p, i, o):
        i.forward_all('_detector')
        o.forward_all('_detector')
        o.forward_all('_info')

    #def configure(self, p, _i, _o):
    #    self._detector_cell = self._detector
        
    def connections(self):
        return [ self._detector ] #self._detector.connections()

########################################################################################################################

class DetectionPipeline:
    '''
    An abstract base class for creating object training pipelines.
    When creating a new detection pipeline, it has to inherit from this one and implement the
    type_name and detector functions.
    '''
    __metaclass__ = ABCMeta

    @classmethod
    def config_doc_default(cls):
        '''
        Return the default documentation for the config file of that detection pipeline
        Do not overload that function
        '''
        return """
               type: '%s'
               module: '%s'
               """ % (cls.type_name(), cls.__module__)

    @classmethod
    def config_doc(cls):
        '''
        Return the documentation for the config file of that detection pipeline
        It should return a string that is interpretable as YAML. It should not contain anything that is standard
        (like the 'module', the name and so on). Anyway, if you use the standard CMake test, it will fail if you do.
        The string should contain the necessary keys. For the values, put anything you want.
        '''
        raise NotImplementedError("The detection pipeline %s must return a YAML string for the configuration docs." %
                                  str(cls))

    @classmethod
    def type_name(cls):
        '''
        Return the code name for your pipeline. eg. 'TOD', 'LINEMOD', 'mesh', etc...
        '''
        raise NotImplementedError("The detection pipeline %s must return a string for the 'name'." % str(cls))

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

########################################################################################################################

def validate_detector(detector):
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

########################################################################################################################

def validate_detection_pipeline(detection_pipeline):
    """
    Makes sure that the detector is valid
    """
    # make sure the docs exist
    doc = detection_pipeline.config_doc()

    # make sure the docs are valid
    try:
        doc_dict = yaml.load(doc)
    except:
        raise RuntimeError("The config documentation for %s is not valid YAML." % str(detection_pipeline))
    if not doc_dict:
        doc_dict = {}
    # make sure there are certain keys in there
    for key in [ 'parameters' ]:
        if key not in doc_dict:
            raise RuntimeError("The config documentation for %s needs the key: '%s'" % (str(detection_pipeline), key))
    # make sure there is no overlap between the necessary docs and the provided docs
    doc_dict_default = yaml.load(detection_pipeline.config_doc_default())
    inter_keys = set(doc_dict_default.keys()).intersection(set(doc_dict.keys()))
    if inter_keys:
        raise RuntimeError('Please remove the following keys from your docs as '
                           'they are handled by the default docs: %s' % str(inter_keys))
