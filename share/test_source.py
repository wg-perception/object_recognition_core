#!/usr/bin/env python
"""
This script tests whether a source can be created given some command line arguments.
It is not meant to be run as a test of object_recognition but as a test for and by each 
module independently.
"""

from object_recognition_core.io.source import SourceBase
from object_recognition_core.utils.find_classes import find_cell
import sys

if __name__ == '__main__':
    source_name = sys.argv[1]

    SourceClass = find_cell([sys.argv[2]], source_name, [SourceBase])

    if len(sys.argv)>=4:
        args = eval(sys.argv[3])
    else:
        args = {}

    source = SourceClass(**args)
    print 'Found source ' + source_name
