#!/usr/bin/env python
"""
Module defining the TOD trainer to train the TOD models
""" 

import ecto
from feature_descriptor import FeatureDescriptor
from ecto_object_recognition import tod_training

########################################################################################################################

class Trainer(ecto.BlackBox):
    def __init__(self, plasm, feature_descriptor_params):
        ecto.BlackBox.__init__(self, plasm)
        self._feature_descriptor_params = feature_descriptor_params
        self.feature_descriptor = FeatureDescriptor(feature_descriptor_params)
        self._depth_to_3d_sparse = calib.DepthTo3dSparse()
        self._keypoints_to_mat = tod_training.KeypointsToMat()
        self._camera_to_world = tod_training.CameraToWorld()
        self._model_stacker = tod_training.TodModelStacker()

    def expose_inputs(self):
        return {'image':self.feature_descriptor['image'],
                'mask':self.feature_descriptor['mask'],
                'depth':self._depth_to_3d_sparse['depth'],
                'K':self._depth_to_3d_sparse['K'],
                'R':self._camera_to_world['R'],
                'T':self._camera_to_world['T']}

    def expose_outputs(self):
        return {'points': self._model_stacker['points'],
                'descriptors': self._model_stacker['descriptors']}

    def expose_parameters(self):
        return {'feature_descriptor_params': self._feature_descriptor_params}

    def connections(self):
        return (self.feature_descriptor['keypoints'] >> self._keypoints_to_mat['keypoints'],
                self._keypoints_to_mat['points'] >> self._depth_to_3d_sparse['points'],
                self._depth_to_3d_sparse['points3d'] >> self._camera_to_world['points'],
                self._camera_to_world['points'] >> self._model_stacker['points'],
                self.feature_descriptor['descriptors'] >> self._model_stacker['descriptors'])
