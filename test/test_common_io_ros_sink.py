#!/usr/bin/env python
from object_recognition_core.io.ros.sink import Publisher
import sys

p = Publisher()

from ecto.opts import CellYamlFactory

assert 'latched' in p.__doc__
assert 'object_ids_topic' in p.__doc__
assert 'pose_topic' in p.__doc__

pub_factory = CellYamlFactory(Publisher, prefix='pub_1')
#to demonstrate that you can have multiple factories.
#this takes on the default values of the instance.
pub_factory2 = CellYamlFactory(Publisher(pose_topic='/poses2', object_ids_topic='/object_ids2'), prefix='pub_2')

with open('pub.yaml', 'w') as f:
    pub_factory.dump(f)
    pub_factory2.dump(f)
    print pub_factory.dump()
    print pub_factory2.dump()

with open('pub.yaml', 'r') as f:
    import yaml
    parsed = yaml.load(f)
    p = pub_factory.load(parsed, cell_name='ros sink')
    p2 = pub_factory2.load(parsed, cell_name='another publisher')
    assert p2.params.pose_topic == '/poses2'
