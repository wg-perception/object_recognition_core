#!/usr/bin/env python
"""
Module defining the TOD trainer to train the TOD models
""" 

from ecto_object_recognition import capture, tod_training
from ecto_opencv import calib
from feature_descriptor import FeatureDescriptor
import ecto

########################################################################################################################

class Trainer(ecto.BlackBox):
    """
    Depth can be of a different size, it will be resized to fit image and mask
    """
    def __init__(self, plasm, feature_descriptor_params):
        ecto.BlackBox.__init__(self, plasm)
        self._feature_descriptor_params = feature_descriptor_params
        self.feature_descriptor = FeatureDescriptor(feature_descriptor_params)
        self._depth_to_3d_sparse = calib.DepthTo3dSparse()
        self._keypoints_to_mat = tod_training.KeypointsToMat()
        self._camera_to_world = tod_training.CameraToWorld()
        self._model_stacker = tod_training.TodModelStacker()
        self._rescale_depth = capture.RescaledRegisteredDepth() #this is for SXGA mode scale handling.
        self._image_duplicator = ecto.Passthrough()

    def expose_inputs(self):
        return {'image':self._image_duplicator['in'],
                'mask':self.feature_descriptor['mask'],
                'depth':self._rescale_depth['depth'],
                'K':self._depth_to_3d_sparse['K'],
                'R':self._camera_to_world['R'],
                'T':self._camera_to_world['T']}

    def expose_outputs(self):
        return {'points': self._model_stacker['points'],
                'descriptors': self._model_stacker['descriptors']}

    def expose_parameters(self):
        return {'feature_descriptor_params': self._feature_descriptor_params}

    def connections(self):
        connections = [self._image_duplicator[:] >> self.feature_descriptor['image'],
                       self._image_duplicator[:] >> self._rescale_depth['image']]
        connections += [self.feature_descriptor['keypoints'] >> self._keypoints_to_mat['keypoints'],
                self._keypoints_to_mat['points'] >> self._depth_to_3d_sparse['points'],
                self._rescale_depth['depth'] >> self._depth_to_3d_sparse['depth'],
                self._depth_to_3d_sparse['points3d'] >> self._camera_to_world['points'],
                self._camera_to_world['points'] >> self._model_stacker['points'],
                self.feature_descriptor['descriptors'] >> self._model_stacker['descriptors']]
        return connections
