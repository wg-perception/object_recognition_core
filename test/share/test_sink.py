#!/usr/bin/env python
"""
This script tests whether a detection pipeline can be created given a config file.
It is not meant to be run as a test of object_recognition but as a test for and by each 
pipeline independently.
"""

from object_recognition_core.utils.find_classes import find_classes
from object_recognition_core.io.sink import Sink
import sys

if __name__ == '__main__':
    sink_name = sys.argv[1]
    if len(sys.argv)>=3:
        args = eval(sys.argv[2])
    else:
        args = {}
    if len(sys.argv)>=4:
        package_names = [ sys.argv[3], 'object_recognition_core.io.sink' ]
    else:
        package_names = [ 'object_recognition_core.io.sink' ]

    sinks = find_classes(package_names, Sink)
    if sink_name not in sinks:
        raise RuntimeError('Invalid sink name: ' + sink_name + '\n'
                           'Make sure that the sink type is defined by a Sink class, in the name class function.\n'
                           'Found sinks: ' + str(sinks))
    sink = sinks[sink_name].sink(**args)
