'''
ABC for Detection pipelines
'''
from object_recognition_core.ecto_cells.io import PipelineInfo
from object_recognition_core.utils.json_helper import obj_to_cpp_json_str
import ecto
from ecto import BlackBoxCellInfo as CellInfo
import warnings

class DetectorBase(object):
    """
    This is a base class for a detection pipeline: you don't need to have your pipeline cell inherit from that class
    but if you do, it will make sure that its inputs/outputs fit the ORK standard (which is good if you want to
    interact with the official ORK pipelines).
    You need to call the BlackBox constructor in your __init__ first and then this function. Typically, your __init__ is
    class Foo(ecto.BlackBox, DetectorBase):
        def __init__(self, *args, **kwargs):
            ecto.BlackBox.__init__(self, *args, **kwargs)
            DetectorBase.__init__(self)
    """
    def __init__(self):
        # check for the pose_results output
        if 'pose_results' not in self.outputs:
            warnings.warn("The detector needs to have a 'pose_results' output tendril.")
        # check for the right type for the pose_results output
        type_name = self.outputs.at('pose_results').type_name
        if type_name not in ['std::vector<object_recognition_core::common::PoseResult, std::allocator<object_recognition_core::common::PoseResult> >']:
            warnings.warn("The detector does not have a 'pose_results' tendril of the right type.\n"
                                      "Must have an output named 'pose_results', with type %s\n"
                                      "This cells output at 'pose_results' has type %s" % ('std::vector<PoseResult>', type_name))

########################################################################################################################

class DetectorAndInfo(ecto.BlackBox):
    """
    This blackbox contains a detection pipeline and a cell that simply forwards its parameters
    Each executed pipeline is actually wrapped in such an object so that outputs can be linked
    to where they are coming from (through the info).
    This is for private implementation
    """
    def __init__(self, detection_class, *args, **kwargs):
        self._detector = detection_class(*args, **kwargs)
        self._info = PipelineInfo(parameters=obj_to_cpp_json_str(kwargs))
        ecto.BlackBox.__init__(self, *args, **kwargs)

    def declare_cells(self, _p):
        return {'detector': self._detector,
                'info': self._info
               }

    @staticmethod
    def declare_forwards(_p):
        return ({'detector': 'all'}, {'detector': 'all'}, {'detector': 'all', 'info': 'all'})

    def connections(self, _p):
        return [ self.detector, self.info ]
