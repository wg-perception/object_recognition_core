#!/usr/bin/env python
from object_recognition.common.io.ros.source import BagReader
from object_recognition.common.io.source import _assert_source_interface
br = BagReader()

assert 'image_message' in br.__doc__
assert 'depth' in br.__doc__
assert 'image' in br.__doc__
assert 'K' in br.__doc__

#verify some types.
assert 'boost::shared_ptr<sensor_msgs::Image_<std::allocator<void> > const>' == br.outputs.at('image_message').type_name

#this should pass our interface check
_assert_source_interface(br)


#test the bag file name parameter
br = BagReader(bag='testy.bag')
assert br.params.bag == 'testy.bag'
assert br._source.params.bag == 'testy.bag'
_assert_source_interface(br)
