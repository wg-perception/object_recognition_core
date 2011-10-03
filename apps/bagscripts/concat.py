#!/usr/bin/env python
import roslib; roslib.load_manifest('rosbag')

import sys
from rosbag import Bag

if len(sys.argv) < 3:
    print >> sys.stderr, 'Usage: concat.py OUTPUT INPUT1 [INPUT2 ...]'
    sys.exit(0)

output_bag = sys.argv[1]
input_bags = sys.argv[2:]

with Bag(output_bag, 'w') as out:
    for input_bag in input_bags:
        for topic, msg, t in Bag(input_bag):
            out.write(topic, msg, t)
