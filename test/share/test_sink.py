#!/usr/bin/env python
"""
This script tests whether a detection pipeline can be created given a config file.
It is not meant to be run as a test of object_recognition but as a test for and by each 
pipeline independently.
"""

from object_recognition_core.utils.find_classes import find_factory
from object_recognition_core.io.sink import Sink, validate_sink
import sys

if __name__ == '__main__':
    sink_name = sys.argv[1]

    sink_factory = find_factory(sys.argv[2], Sink, sink_name)

    if len(sys.argv)>=4:
        args = eval(sys.argv[3])
    else:
        args = {}

    sink = sink_factory.sink(**args)
    validate_sink(sink)
    print 'Found sink ' + sink_name
