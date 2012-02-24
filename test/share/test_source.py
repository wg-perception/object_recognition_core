#!/usr/bin/env python
"""
This script tests whether a detection pipeline can be created given a config file.
It is not meant to be run as a test of object_recognition but as a test for and by each 
pipeline independently.
"""

from object_recognition_core.utils.find_classes import find_classes
from object_recognition_core.io.source import Source
import sys

if __name__ == '__main__':
    source_name = sys.argv[1]
    if len(sys.argv)>=3:
        args = eval(sys.argv[2])
    else:
        args = {}
    if len(sys.argv)>=4:
        package_names = [ sys.argv[3], 'object_recognition_core.io.source' ]
    else:
        package_names = [ 'object_recognition_core.io.source' ]

    sources = find_classes(package_names, Source)
    if source_name not in sources:
        raise RuntimeError('Invalid source name: ' + source_name + '\n'
                           'Make sure that the source type is defined by a Source class, in the name class function.\n'
                           'Found sources: ' + str(sources))
    source = sources[source_name].source(**args)
