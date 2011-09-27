#!/usr/bin/env python
"""
Module defining the TOD trainer to train the TOD models
"""

from ecto_object_recognition import capture, tod_training, conversion
from ecto_opencv import calib, features2d, highgui
from ecto_g2o import SBA
from feature_descriptor import FeatureDescriptor
import ecto
from ecto_X import Executer

########################################################################################################################

class Trainer(ecto.BlackBox):
    """
    Depth can be of a different size, it will be resized to fit image and mask
    """
    def __init__(self, plasm, input_cell, json_params, display=False):
        ecto.BlackBox.__init__(self, plasm)

        self._input_cell = input_cell
        self._display = display

        self._feature_descriptor = FeatureDescriptor(json_params['feature_descriptor'])
        self._depth_to_3d_sparse = calib.DepthTo3dSparse()
        self._executer = None
        self._keypoints_to_mat = tod_training.KeypointsToMat()
        self._camera_to_world = tod_training.CameraToWorld()
        self._model_stacker = tod_training.TodModelStacker()
        self._point_merger = tod_training.PointMerger()
        self._prepare_for_g2o = tod_training.PrepareForG2O()
        self._rescale_depth = capture.RescaledRegisteredDepth() #this is for SXGA mode scale handling.
        self._keypoint_validator = conversion.KeypointsValidator()
        self._g2o = SBA()

        self._image_duplicator = ecto.Passthrough()
        self._mask_duplicator = ecto.Passthrough()
        self._K_duplicator = ecto.Passthrough()
        self._depth_duplicator = ecto.Passthrough()

    def expose_inputs(self):
        return {'image':self._image_duplicator['in'],
                'mask':self._mask_duplicator['in'],
                'depth':self._depth_duplicator['in'],
                'K':self._K_duplicator['in'],
                'R':self._camera_to_world['R'],
                'T':self._camera_to_world['T']}

    def expose_outputs(self):
        return {'points': self._point_merger['points'],
                'descriptors': self._point_merger['descriptors']}

    def expose_parameters(self):
        return {}

    def connections(self):
        sub_connections = [self._image_duplicator[:] >> self._feature_descriptor['image'],
                           self._image_duplicator[:] >> self._rescale_depth['image'],
                           self._mask_duplicator[:] >> self._feature_descriptor['mask'],
                           self._depth_duplicator[:] >> self._rescale_depth['depth'],
                           self._K_duplicator[:] >> self._depth_to_3d_sparse['K']]
        # Make sure the keypoints are in the mask and with a valid depth
        sub_connections += [self._feature_descriptor['keypoints', 'descriptors'] >>
                            self._keypoint_validator['keypoints', 'descriptors'],
                            self._K_duplicator[:] >> self._keypoint_validator['K'],
                            self._mask_duplicator[:] >> self._keypoint_validator['image'],
                            self._rescale_depth['depth'] >> self._keypoint_validator['image'] ]
        # transform the keypoints/depth into 3d points
        sub_connections += [ self._keypoint_validator['points'] >> self._depth_to_3d_sparse['points'],
                        self._rescale_depth['depth'] >> self._depth_to_3d_sparse['depth'],
                        self._depth_to_3d_sparse['points3d'] >> self._camera_to_world['points']]
        # store all the info
        sub_connections += [ self._camera_to_world['points3d'] >> self._model_stacker['points3d'],
                        self._keypoint_validator['points'] >> self._model_stacker['points'],
                        self._keypoint_validator['descriptors'] >> self._model_stacker['descriptors'] ]

        if self._display:
            mask_view = highgui.imshow(name="mask")
            depth_view = highgui.imshow(name="Depth");

            sub_connections += [ self._mask_duplicator[:] >> mask_view['image'],
                            self._depth_duplicator[:] >> depth_view['image']]
            # draw the keypoints
            keypoints_view = highgui.imshow(name="Keypoints")
            draw_keypoints = features2d.DrawKeypoints()
            sub_connections += [ self._image_duplicator[:] >> draw_keypoints['image'],
                            self._feature_descriptor['keypoints'] >> draw_keypoints['keypoints'],
                            draw_keypoints['image'] >> keypoints_view['image']]

        plasm = ecto.Plasm()
        plasm.connect(sub_connections)
        executer = Executer(plasm=plasm, niter=0, outputs={'point3d':self._model_stacker,
                            'points':self._model_stacker, 'descriptors':self._model_stacker, 'Ks': self._model_stacker,
                            'quaternions': self._model_stacker, 'Ts': self._model_stacker})
 
        connections = [ executer['Ks', 'quaternions', 'Ts'] >> self._g2o['Ks', 'quaternions', 'Ts'],
                       executer['points3d', 'points'] >> self._prepare_for_g2o['points3d', 'points'],
                       self._prepare_for_g2o['points3d', 'points'] >>self._g2o['points3d', 'points'],
                       self._g2o['points'] >> self._point_merger['points'],
                       self._prepare_for_g2o['ids'] >> self._point_merger['ids'],
                       self._executer['descriptors'] >> self._point_merger['descriptors'] ]
 
        return connections
