#!/usr/bin/env python
from object_recognition.common.io.source import _assert_source_interface

#test that names have to be correct.
try:
    from ecto import And
    a = And()
    _assert_source_interface(a)
    assert False == "Should not have gotten here."
except NotImplementedError,e:
    assert "Must have an output named K" in str(e)


import ecto
outputs = ecto.Tendrils()
outputs.declare("image","an image", "image image")
outputs.declare("depth","a depth map", "depth depth")
outputs.declare("K","a camera matrix", "eye(3)")
outputs.declare("points3d","A matrix of 3 vectors", "Mat(N,3)")

class Source(object):
    pass
kr = Source()
kr.outputs = outputs
try:
    _assert_source_interface(kr)
    assert False == "Should not have gotten here."
except NotImplementedError,e:
    assert "This cells output at K has type boost::python::api::object" in str(e)

