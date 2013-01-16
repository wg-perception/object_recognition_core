#!/usr/bin/env python
"""
This script tests whether a sink can be created given some command line arguments.
It is not meant to be run as a test of object_recognition but as a test for and by each 
module independently.
"""

from object_recognition_core.io.sink import SinkBase
from object_recognition_core.utils.find_classes import find_cell
import sys

if __name__ == '__main__':
    sink_name = sys.argv[1]

    SinkClass = find_cell([sys.argv[2]], sink_name, [SinkBase])

    if len(sys.argv)>=4:
        args = eval(sys.argv[3])
    else:
        args = {}

    sink = SinkClass(**args)
    print 'Found sink ' + sink_name
