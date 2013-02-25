'''
ABC for Detection pipelines
'''
from ecto import BlackBoxCellInfo as CellInfo
from object_recognition_core.ecto_cells.io import PipelineInfo
from object_recognition_core.utils.json_helper import obj_to_cpp_json_str
import ecto

class DetectorBase(object):
    """
    This is a base class for a detection pipeline: you don't need to have your pipeline cell inherit from that class
    but if you do, it will make sure that its inputs/outputs fit the ORK standard (which is good if you want to
    interact with the official ORK pipelines).
    You need to call the BlackBox constructor in your __init__ first and then this function. Typically, your __init__ is

        >>> class Foo(ecto.BlackBox, DetectorBase):
        >>>    def __init__(self, *args, **kwargs):
        >>>        ecto.BlackBox.__init__(self, *args, **kwargs)
        >>>        DetectorBase.__init__(self)
    """
    def __init__(self, do_check_object_ids=True, do_check_db=True):
        """
        Normal constructor that checks for the validity of inputs/outputs
        
        :param do_check_object_ids: if True, it is checked that the cell has an input 'json_object_ids' that is a string
        :param do_check_db: if True, it is checked that the cell has an input 'json_db' that is a string
        """
        checks = {'output': [ ('pose_results', ['std::vector<object_recognition_core::common::PoseResult, std::allocator<object_recognition_core::common::PoseResult> >'], 'std::vector<PoseResult>'),
                             ],
                  'input': [],
                  'param': []}
        if do_check_object_ids:
            checks['param'].append(('json_object_ids', ['std::string', 'boost::python::api::object'], 'std::string'))
        if do_check_db:
            checks['param'].append(('json_db', ['std::string', 'boost::python::api::object'], 'std::string'))

        for check_type, check_list in list(checks.items()):
            tendrils = {'input':self.inputs, 'output':self.outputs, 'param':self.params}[check_type]
            for check in check_list:
                tendril_name, tendril_types_cpp, tendril_types_print = check
                if tendril_name not in tendrils:
                    raise RuntimeError('The detector needs to have a "%s" %s tendril.' % (tendril_name, check_type))
                # check for the right type for the pose_results output
                type_name = tendrils.at(tendril_name).type_name
                if type_name not in tendril_types_cpp:
                    raise RuntimeError('The detector does not have a "%s" tendril of the right type.\n' % tendril_name +
                                    'Must have a %s named "%s", with type "%s"\n and not "%s"' % (check_type,
                                                                        tendril_name, tendril_types_print, type_name))

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
