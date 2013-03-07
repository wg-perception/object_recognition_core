"""
This file implements functions to help documentation:
- autogeneration of config files
"""
from object_recognition_core.utils.find_classes import find_classes
import ecto
import os

def config_yaml_for_ecto_cell(cls, header=None):
    """
    Given an ecto cell, generate YAML for all the possibles parameters

    :param cls: the class of an ecto cell
    :param header: this is just the name of the cell section. If None, no header is written and no indent is given
    """
    if header:
        res = '%s:\n' % header
        indent = '   '
    else:
        res = ''
        indent = ''
    res += '%stype: %s\n' % (indent, cls.__name__)
    res += '%smodule: %s\n' % (indent, cls.__module__)
    # display the parameters
    res += '%sparameters:\n' % indent
    p = ecto.Tendrils()
    try:
        cls.declare_params(p)
    except AttributeError:
        p = cls.params
    for tendril_name, tendril in list(p.items()):
        # Split the doc string to 100 characters
        line = '%s   # ' % indent
        for word in tendril.doc.split():
            if len(line + ' ' + word) > 100:
                res += line + '\n'
                line = '%s   # %s' % (indent, word)
            else:
                line += ' ' + word
        res += line + '\n'
        res += '%s   %s: %s\n' % (indent, tendril_name, tendril.val)

    return res

########################################################################################################################

def config_yaml_for_ecto_cells(class_type):
    """
    Function returning an array of doc strings for each cell of class `class_type` in object_recognition
    :param class_type: one of 'detection_pipeline', 'training_pipeline', 'source', 'sink'
    """
    from object_recognition_core.io.sink import SinkBase
    from object_recognition_core.io.source import SourceBase
    from object_recognition_core.io.voter import VoterBase
    from object_recognition_core.pipelines.detection import DetectorBase
    from object_recognition_core.pipelines.training import TrainerBase

    supported_classes = {'detection_pipeline': DetectorBase, 'training_pipeline': TrainerBase,
                         'source': SourceBase, 'sink': SinkBase}
    if class_type not in supported_classes:
        raise RuntimeError('Class type not support: %s. Accepted are: %s' % (class_type,
                                                                             str(supported_classes.keys())))

    modules = set()
    # go over the modules on the PYTHONPATH and only keep the ones that start with object_recognition
    if 'PYTHONPATH' not in os.environ:
        raise RuntimeError('You need a PYTHONPATH to use that script')
    for path in os.environ['PYTHONPATH'].split(':'):
        if not os.path.isdir(path):
            continue
        for name in os.listdir(path):
            if os.path.isdir(os.path.join(path,name)) and (name.startswith('object_recognition') or name.startswith('ork')):
                modules.add(name)
    # find all the objects of the right type
    classes = find_classes(modules, [supported_classes[class_type]])

    # create a string with the config documentation
    res_list = []

    class_number = 0
    for class_object in classes:
        res = config_yaml_for_ecto_cell(class_object, '%s_%s' % (class_type, class_number))

        class_number += 1
        res_list.append(res)

    return res_list
