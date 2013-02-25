'''
Function that finds classes of a certain base type on the path in certain modules
'''

import inspect
import os
import sys

class CellNotFound(Exception):
    '''
    Error class specific to when a certain factory is not found
    '''
    def __init__(self, value):
        self.value = value
        Exception.__init__(self)

    def __str__(self):
        return repr(self.value)

########################################################################################################################

def find_classes(modules, base_types):
    """
    Given a list of python packages, or modules, find all implementations of a class.

    :param modules: The names of the modules to look into
    :param base_type: any class type (TrainingPipeline, Sink ...). Can be empty.
    :returns: A set of found classes
    """
    classes = set()
    modules_loaded = []
    module_names = set()
    for module in modules:
        if module == '':
            continue

        m = __import__(module)
        modules_loaded.append(sys.modules[module])

        path_list = m.__path__
        if not isinstance(path_list, list):
            path_list = [ path_list ]
        for module_path in path_list:
            path_len = len(os.path.split(module_path)[0])
            for root, dirs, files in os.walk(module_path):
                # record the files as being modules
                for name in files:
                    if name == '__init__.py':
                        package_name = os.path.split(module_path)[0][path_len+1:].replace(os.path.sep,'.')
                        if package_name:
                            module_names.add(package_name)
                    elif name.endswith('.py') or name.endswith('.so'):
                        path = os.path.join(root, name)
                        module_names.add(path[path_len+1:-3].replace(os.path.sep,'.'))
                # record the files as being modules
                for directory in dirs:
                    path = os.path.join(root, directory)
                    module_names.add(path[path_len+1:].replace(os.path.sep,'.'))

    for module_name in module_names:
        try:
            m = __import__(module_name, fromlist=[module_name])
            modules_loaded.append(m)
        except:
            continue

    # Look into the modules
    for pymodule in modules_loaded:
        for _name, potential_pipeline in inspect.getmembers(pymodule):
            # check if an object is a class
            if not inspect.isclass(potential_pipeline):
                continue
            # make sure the class is from the right module (with recursion, weird things can happen)
            if not any([potential_pipeline.__module__.startswith(module_name) for module_name in modules]):
                continue
            # check if an object is a class that is none of the sought ones
            if potential_pipeline not in base_types:
                # make sure the class is a subclass of the base types
                if not base_types or any([ issubclass(potential_pipeline, base_type) for base_type in base_types]):
                    classes.add(potential_pipeline)

    return classes

########################################################################################################################

def find_cells(modules, base_types=None):
    """
    Given a list of python packages, or modules, find all ecto cells that also inherit from some base types.

    :param modules: The names of the modules to look into
    :param base_types: a list of any class type (TrainingPipeline, Sink ...)
    :returns: A set of found cell classes
    """
    if base_types is None:
        potential_cells = find_classes(modules, [])
    else:
        potential_cells = find_classes(modules, base_types)

    cells = set([cell_class for cell_class in potential_cells if getattr(cell_class, '__looks_like_a_cell__', False) ])

    return cells

########################################################################################################################

def __find_unique_class(class_name, classes, base_types, modules):
    """
    This is a convenience function for that module. It finds a class of a given name in a set of classes
    
    :param class_name: the name of the class to find
    :param classes: a set of classes to look into
    :param base_types: the base types of that class. Can be None, it's only for error report
    :param modules: the modules in which to look. Can be [], it's only for error report
    """
    class_class_final = None
    for class_class in classes:
        if class_class.__name__ == class_name:
            if class_class_final is None:
                class_class_final = class_class
            else:
                raise CellNotFound('Already found a cell of name "%s" in addition to %s : %s' % 
                               (class_name, str(class_class), str(class_class_final)))
    if class_class_final:
        return class_class_final
    else:
        if base_types:
            raise CellNotFound('Could not find ecto cell/BlackBox of name "%s" and also of type %s ' % 
                               (class_name, base_types) + 'in modules "%s"' % str(modules))
        else:
            raise CellNotFound('Could not find ecto cell/BlackBox of name "%s" in modules "%s".' % (class_name,
                                                                                                    str(modules)))

def find_class(modules, class_name, base_types=None):
    """
    Given a list of python packages, or modules, find a cell of a given name and possible inheritance can be given too

    :param modules: The names of the modules to look into
    :param class_name: The name of the ecto cell to look for
    :param base_types: a list of any class type (TrainingPipeline, Sink ...)
    :returns: A dictionary of found classes: the key is the class name and the value the object class itself
    """
    classes = find_cells(modules, base_types)

    return __find_unique_class(class_name, classes, base_types, modules)

########################################################################################################################

def find_cell(modules, cell_name, base_types=None):
    """
    Given a list of python packages, or modules, find an ecto cell/BlackBox. This cell can also inherit from
    a specific base type of specified

    :param modules: The names of the modules to look into
    :param cell_name: The name of the ecto cell to look for
    :param base_types: a list of any class type (TrainingPipeline, Sink ...)
    :returns: A dictionary of found classes: the key is the class name and the value the object class itself
    """
    cells = find_cells(modules, base_types)

    return __find_unique_class(cell_name, cells, base_types, modules)
