#!/usr/bin/env python
from object_recognition.common.io.ros.source import KinectReader
from object_recognition.common.io.source import _assert_source_interface
kr = KinectReader()

assert 'image_message' in kr.__doc__
assert 'depth' in kr.__doc__
assert 'image' in kr.__doc__
assert 'K' in kr.__doc__

#verify some types.
assert 'boost::shared_ptr<sensor_msgs::Image_<std::allocator<void> > const>' == kr.outputs.at('image_message').type_name

#this should pass our interface check
_assert_source_interface(kr)
