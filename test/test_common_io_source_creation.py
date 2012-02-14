#!/usr/bin/env python
from object_recognition_core.io.source import _assert_source_interface
from object_recognition_core.io.source import Source

import argparse
parser = argparse.ArgumentParser(description='An source reader.')
Source.add_arguments(parser)
parser.print_help()

#test default args
args = parser.parse_args()
source = Source.parse_arguments(args)
_assert_source_interface(source)
print source.__class__.__name__

#assert 'KinectReader' == source.__class__.__name__

#test a bad bag
args = parser.parse_args(['--ros_bag','non_existant.bag'])
try:
    source = Source.parse_arguments(args)
except RuntimeError,e:
    print str(e)
    assert 'non_existant.bag does not exist.' in str(e)

#test an existing bag
with open('a_test.bag','w') as f:
    f.write('\ntest\n')

args = parser.parse_args(['--ros_bag','a_test.bag'])
#note that the bag hasn't been opened here.
source = Source.parse_arguments(args)
_assert_source_interface(source)
assert 'BagReader' == source.__class__.__name__
