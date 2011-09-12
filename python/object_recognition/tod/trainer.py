#!/usr/bin/env python
"""
Module defining the TOD trainer to train the TOD models
"""

from ecto_object_recognition import capture, tod_training
from ecto_opencv import calib, highgui
from feature_descriptor import FeatureDescriptor
import ecto

########################################################################################################################

class Trainer(ecto.BlackBox):
    """
    Depth can be of a different size, it will be resized to fit image and mask
    """
    def __init__(self, plasm, json_params, display=False):
        ecto.BlackBox.__init__(self, plasm)

        self._display = display

        self._feature_descriptor = FeatureDescriptor(json_params['feature_descriptor'])
        self._depth_to_3d_sparse = calib.DepthTo3dSparse()
        self._keypoints_to_mat = tod_training.KeypointsToMat()
        self._camera_to_world = tod_training.CameraToWorld()
        self._model_stacker = tod_training.TodModelStacker()
        self._rescale_depth = capture.RescaledRegisteredDepth() #this is for SXGA mode scale handling.

        self._image_duplicator = ecto.Passthrough()
        self._mask_duplicator = ecto.Passthrough()
        self._depth_duplicator = ecto.Passthrough()

    def expose_inputs(self):
        return {'image':self._image_duplicator['in'],
                'mask':self._mask_duplicator['in'],
                'depth':self._depth_duplicator['in'],
                'K':self._depth_to_3d_sparse['K'],
                'R':self._camera_to_world['R'],
                'T':self._camera_to_world['T']}

    def expose_outputs(self):
        return {'points': self._model_stacker['points'],
                'descriptors': self._model_stacker['descriptors']}

    def expose_parameters(self):
        return {}

    def connections(self):
        connections = [self._image_duplicator[:] >> self._feature_descriptor['image'],
                       self._image_duplicator[:] >> self._rescale_depth['image'],
                       self._mask_duplicator[:] >> self._feature_descriptor['mask'],
                       self._depth_duplicator[:] >> self._rescale_depth['depth']]
        connections += [self._feature_descriptor['keypoints'] >> self._keypoints_to_mat['keypoints'],
                self._keypoints_to_mat['points'] >> self._depth_to_3d_sparse['points'],
                self._rescale_depth['depth'] >> self._depth_to_3d_sparse['depth'],
                self._depth_to_3d_sparse['points3d'] >> self._camera_to_world['points'],
                self._camera_to_world['points'] >> self._model_stacker['points'],
                self._feature_descriptor['descriptors'] >> self._model_stacker['descriptors']]

        if self._display:
            image_view = highgui.imshow(name="RGB")
            mask_view = highgui.imshow(name="mask")
            depth_view = highgui.imshow(name="Depth");
            connections += [self._image_duplicator[:] >> image_view['image'],
                          self._mask_duplicator[:] >> mask_view['image'],
                          self._depth_duplicator[:] >> depth_view['image']]

        return connections
