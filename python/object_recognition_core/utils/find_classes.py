'''
Function that finds classes of a certain base type on the path in certain modules
'''
import inspect
import pkgutil
import sys

def find_classes(modules, base_type):
    '''
    Given a list of python packages, or modules, find all implementations.
    :param modules: The names of the modules to look into
    :param base_type: any class type (TrainingPipeline, Sink ...)
    :returns: A dictionary of found classes: the key is the class name and the value the object class itself
    '''
    pipelines = {}
    ms = []
    for module in modules:
        if module == '':
            continue

        m = __import__(module)
        ms += [m]
        for loader, module_name, is_pkg in  pkgutil.walk_packages(m.__path__):
            if is_pkg:
                module = loader.find_module(module_name).load_module(module_name)
                ms.append(module)

    for pymodule in ms:

        for name, potential_pipeline in inspect.getmembers(pymodule):
            if inspect.isclass(potential_pipeline) and potential_pipeline != base_type and \
                                                                issubclass(potential_pipeline, base_type):
                pipelines[potential_pipeline.type_name()] = potential_pipeline
    return pipelines
