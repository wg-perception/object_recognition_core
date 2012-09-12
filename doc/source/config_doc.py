#!/usr/bin/env python
"""
This file crawls the PYTHONPATH and returns the config parameters of a class used in
a graph. Those parameters could be copied/pasted into a config file for ORK.

"""

from object_recognition_core.io.sink import Sink
from object_recognition_core.io.source import Source
from object_recognition_core.io.voter import Voter
from object_recognition_core.pipelines.detection import DetectionPipeline
from object_recognition_core.pipelines.training import TrainingPipeline
from object_recognition_core.utils.find_classes import find_classes
import argparse
import json
import os
import yaml

def indent_yaml(yaml_str):
    """
    Receive a string, and make sure the indentation of the top levels is 0
    """
    # figure out the smallest indent we have
    for line in yaml_str.split('\n'):
        if not line.strip():
            continue
        if line.strip().startswith('#'):
            continue
        min_indent = 0
        for char in line:
            if char == ' ':
                min_indent += 1
            else:
                break
        break
    # clean the original string
    final_yaml_str = ''
    for line in yaml_str.split('\n'):
        if not line.strip():
            continue
        if line.strip().startswith('#'):
            new_line = line[min_indent:]
            if new_line.strip().startswith('#'):
                final_yaml_str += new_line
            else:
                final_yaml_str += line
        else:
            final_yaml_str += line[min_indent:]
        final_yaml_str += '\n'

    return final_yaml_str

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('class_type', type=str, help='The class to get instance configs of')
    parser.add_argument('class_name', type=str, help='The names of the classes to get instance configs of', nargs='*')
    args = parser.parse_args()

    supported_classes = {'detection_pipeline': DetectionPipeline, 'training_pipeline': TrainingPipeline,
                         'source': Source, 'sink': Sink}
    if args.class_type not in supported_classes:
        raise RuntimeError('Class type not support: %s. Accepted are: %s' % (args.class_type,
                                                                             str(supported_classes.keys())))

    modules = set()
    # go over the modules on the PYTHONPATH and only keep the ones that start with object_recognition
    if 'PYTHONPATH' not in os.environ:
        raise RuntimeError('You need a PYTHONPATH to use that script')
    for path in os.environ['PYTHONPATH'].split(':'):
        if not os.path.isdir(path):
            continue
        for name in os.listdir(path):
            if os.path.isdir(name) and name.startswith('object_recognition'):
                modules.add(name)
    # find all the objects of the right type
    classes = find_classes(modules, supported_classes[args.class_type])

    # create a string with the config documentation
    res = ''
    class_number = 0
    for _class_name, class_object in classes.items():
        res += '[%s%s]\n' % (args.class_type, class_number)
        res += indent_yaml(class_object.config_doc_default())
        if yaml.load(class_object.config_doc()):
            res += indent_yaml(class_object.config_doc())
        class_number += 1
        res += '\n'

    print res[:-1]
    #json.dumps(doc,final_doc,indent=0)
