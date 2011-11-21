"""
Module defining several inputs for the object recognition pipeline
""" 
import ecto
from ecto_opencv import calib
from ecto_object_recognition import capture
from ecto_openni import OpenNICapture, DEPTH_RGB, DEPTH_IR, RGB, IR, IRGamma, enumerate_devices

########################################################################################################################

class OpenNISource(ecto.BlackBox):
    """A source that uses ROS to produce image, depth, and camera info.
    """
    _depth_map = capture.RescaledRegisteredDepth
    _points3d = calib.DepthTo3d
    _source = OpenNICapture
    _depth_mask = calib.DepthMask

    def declare_params(self, p):
        p.forward_all('_source')

    def declare_io(self, _p, _i, o):
        o.forward('image', cell_name='_source', cell_key='image',
                  doc='The RGB image from a OpenNI device.')
        o.forward('depth', cell_name='_depth_map', cell_key='depth',
                  doc='The depth map from a OpenNI device.'+
                  ' This is a CV_32FC1, with values in meters.')
        o.forward('K', cell_name='_source', cell_key='K',
                  doc='The camera intrinsics matrix.')
        o.forward('points3d', cell_name='_points3d')
        o.forward('mask',cell_name='_depth_mask')


    def configure(self, p, _i, _o):
        #these transform the depth into something usable
        self._depth_map = OpenNISource._depth_map()
        self._points3d = OpenNISource._points3d()

    def connections(self):
        #rescaling ...
        graph = [self._source['depth'] >> self._depth_map['depth'],
                  self._source['image'] >> self._depth_map['image'],
                  ]
        
        #depth ~> 3d calculations
        graph += [
                  self._depth_map['depth'] >> self._points3d['depth'],
                  self._source['K'] >> self._points3d['K'],
                  self._depth_map['depth'] >> self._depth_mask['depth']
                 ]
        
        return graph
