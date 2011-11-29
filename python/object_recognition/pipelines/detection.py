'''
ABC for Detection pipelines
'''
from abc import ABCMeta, abstractmethod
from ecto_object_recognition.object_recognition_db import DbModels
from object_recognition.common.io.sink import Sink
from object_recognition.common.io.source import Source
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

    def source(self, source_params):
        return Source.parse_arguments(source_params)

    def sink(self, sink_params, object_ids, db_params):
        return Sink.parse_arguments(sink_params, db_params, object_ids)

    def detector(self, pipeline_subtype, pipeline_params, db_params, model_documents, args):
        '''
        Returns the detector cell
        '''
        raise NotImplementedError("The detector has to be implemented.")

    @classmethod
    def detect(cls, source_params, sink_params, pipeline_subtype, pipeline_params, db_params, object_ids, args=None):
        '''
        Returns a training plasm, that will be executed exactly once.
        :param object_id: The object id to train up.
        :param session_ids: A list of session ids that this model should be based on.
        :param observation_ids: A list of observation ids that will be dealt to the incremental model builder.
        :param pipeline_params: A dictionary of parameters that will be used to initialize the training pipeline.
        :param db_params: A DB parameters object that specifies where to save the model to.
        :param args: General command line args, for things like visualize or what have you.
        :returns: A plasm, only execute once please.
        '''
        pipeline = cls()

        model_documents = pipeline.db_models(object_ids, pipeline_subtype, db_params)

        source = pipeline.source(source_params)
        sink = pipeline.sink(sink_params, object_ids, db_params)
        detector = pipeline.detector(pipeline_subtype, pipeline_params, db_params, model_documents, args)

        plasm = ecto.Plasm()
        # Connect the source to the detector
        for key in set(source.outputs.keys()).intersection(detector.inputs.keys()):
            plasm.connect(source[key] >> detector[key])

        # Connect the detector to the sink
        for key in set(detector.outputs.keys()).intersection(sink.inputs.keys()):
            plasm.connect(detector[key] >> sink[key])
        
        # TODO fix that
        # make sure that we also give the image_message, in case we want to publish a topic
        if 'image_message' in sink.inputs and 'image_message' in source.outputs:
            plasm.connect(source['image_message'] >> sink['image_message'])

        return plasm
