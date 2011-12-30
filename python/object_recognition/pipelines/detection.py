'''
ABC for Detection pipelines
'''
from abc import ABCMeta, abstractmethod
from object_recognition.common.utils import json_helper
import ecto

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

    def db_models(self, object_ids, submethod, db_params):
        return DbModels(db_params, object_ids, self.type_name(), json_helper.dict_to_cpp_json_str(submethod))

    @classmethod
    def detector(cls, *args, **kwargs):
        '''
        Returns the detector cell
        '''
        raise NotImplementedError("The detector has to be implemented.")
