'''
Loaders for all object recognition pipelines
'''
from abc import ABCMeta, abstractmethod
import inspect
import pkgutil
from object_recognition.pipelines.detection import DetectionPipeline
from object_recognition.pipelines.training import TrainingPipeline

def find_pipelines(modules, pipeline_type):
    '''
    Given a list of python packages, or modules, find all TrainingPipeline implementations.
    :param modules: A list of python package names, e.g. ['object_recognition']
    :returns: A list of TrainingPipeline implementation classes.
    '''
    pipelines = {}
    ms = []
    for module in modules:
        m = __import__(module)
        ms += [(module, m)]
        for loader, module_name, is_pkg in  pkgutil.walk_packages(m.__path__):
            if is_pkg:
                module = loader.find_module(module_name).load_module(module_name)
                ms.append(module)
    for pymodule in ms:
        for x in dir(pymodule):
            potential_pipeline = getattr(pymodule, x)
            if inspect.isclass(potential_pipeline) and potential_pipeline != pipeline_type and \
                                                            issubclass(potential_pipeline, pipeline_type):
                pipelines[potential_pipeline.type_name()] = potential_pipeline
    return pipelines
