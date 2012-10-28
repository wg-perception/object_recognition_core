#!/usr/bin/env python
"""
This script tests whether a detection pipeline can be created given a config file.
It is not meant to be run as a test of object_recognition but as a test for and by each 
pipeline independently.
"""

from object_recognition_core.utils.find_classes import find_factory
from object_recognition_core.io.source import Source, validate_source
import sys

if __name__ == '__main__':
    source_name = sys.argv[1]

    source_factory = find_factory(sys.argv[2], Source, source_name)

    if len(sys.argv)>=4:
        args = eval(sys.argv[3])
    else:
        args = {}

    source = source_factory.source(**args)
    validate_source(source)
    print 'Found source ' + source_name
