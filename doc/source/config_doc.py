#!/usr/bin/env python
"""
This file crawls the PYTHONPATH and returns the config parameters of a class used in
a graph. Those parameters could be copied/pasted into a config file for ORK.

"""

from __future__ import print_function
from object_recognition_core.io.sink import SinkBase
from object_recognition_core.io.source import SourceBase
from object_recognition_core.io.voter import VoterBase
from object_recognition_core.pipelines.detection import DetectorBase
from object_recognition_core.pipelines.training import TrainerBase
from object_recognition_core.utils.find_classes import find_classes
from object_recognition_core.utils.doc import config_yaml_for_ecto_cell
import argparse
import ecto
import json
import os
import yaml

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('class_type', type=str, help='The class to get instance configs of')
    parser.add_argument('class_name', type=str, help='The names of the classes to get instance configs of', nargs='*')
    args = parser.parse_args()

    supported_classes = {'detection_pipeline': DetectorBase, 'training_pipeline': TrainerBase,
                         'source': SourceBase, 'sink': SinkBase}
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
            if os.path.isdir(os.path.join(path,name)) and (name.startswith('object_recognition') or name.startswith('ork')):
                modules.add(name)
    # find all the objects of the right type
    classes = find_classes(modules, [supported_classes[args.class_type]])

    # create a string with the config documentation
    res_list = []

    class_number = 0
    for class_object in classes:
        res = config_yaml_for_ecto_cell(class_object, '%s_%s' % (args.class_type, class_number))

        class_number += 1
        res_list.append(res)

    print('\n'.join(res_list))
