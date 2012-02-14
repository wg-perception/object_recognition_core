import inspect
import pkgutil

def find_cells(modules, pipeline_type):
    '''
    Given a list of python packages, or modules, find all implementations.
    :param modules: The names of the modules to look into
    :param pipeline_type: any class type (TrainingPipeline, Sink ...)
    :returns: A list of TrainingPipeline implementation classes.
    '''
    pipelines = {}
    ms = []
    for module in modules:
        m = __import__(module)
        ms += [m]
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
