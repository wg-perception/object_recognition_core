'''
Function that finds classes of a certain base type on the path in certain modules
'''
import inspect
import pkgutil

class FactoryNotFound(Exception):
    '''
    Error class specific to when a certain factory is not found
    '''
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

#########################################################################################

def find_factory(module, base_type, type_name):
    '''
    Given a list of python  modules, find all implementations of a class.
    :param modules: The names of the modules to look into
    :param base_type: any factory class type (TrainingPipeline, Sink ...)
    :returns: A dictionary of found classes: the key is the class.type_name() and the value the object class itself
    '''   
    if module == '':
        return

    m = __import__(module)

    pipelines = {}  
    for factory in base_type.__subclasses__():
        if factory.type_name() == type_name:
            return factory

    raise FactoryNotFound('Could not find a factory that is a subclass of "%s" with type_name() "%s" in module "%s"' %
                          (str(base_type), type_name, module))

#########################################################################################

def find_factories(modules, base_type):
    '''
    Given a list of python  modules, find all implementations of a class.
    :param modules: The names of the modules to look into
    :param base_type: any factory class type (TrainingPipeline, Sink ...)
    :returns: A dictionary of found classes: the key is the class.type_name() and the value the object class itself
    '''
    for module in modules:
        if module == '':
            continue

        m = __import__(module)

    pipelines = {}
    for factory in base_type.__subclasses__():
        pipelines[factory.type_name()] = factory

    return pipelines

#########################################################################################

def find_classes_deprecated(modules, base_type):
    '''
    This function was actually too generic as what always mattered was to find a subclass of a factory
    Given a list of python packages, or modules, find all implementations of a class.
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
                try:
                    module = loader.find_module(module_name).load_module(module_name)
                    ms.append(module)
                except ImportError:
                    continue

    for pymodule in ms:
        for _name, potential_pipeline in inspect.getmembers(pymodule):
            if inspect.isclass(potential_pipeline) and potential_pipeline != base_type and \
                                                                issubclass(potential_pipeline, base_type):
                pipelines[potential_pipeline.type_name()] = potential_pipeline
    return pipelines
