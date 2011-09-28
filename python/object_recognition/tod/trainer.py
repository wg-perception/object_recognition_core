#!/usr/bin/env python
"""
Module defining the TOD trainer to train the TOD models
"""

from ecto_object_recognition import capture, tod_training, conversion
from ecto_opencv import calib, features2d, highgui
from g2o import SBA
from feature_descriptor import FeatureDescriptor
import ecto
from ecto_X import Executer
from object_recognition.common.utils.json_helper import dict_to_cpp_json_str

########################################################################################################################

class Trainer(ecto.BlackBox):
    """
    Depth can be of a different size, it will be resized to fit image and mask
    """
    def __init__(self, plasm, input_cell, obs_ids, json_params, display=False):
        ecto.BlackBox.__init__(self, plasm)

        self._input_cell = input_cell
        self._display = display
        self._obs_ids = obs_ids

        self._feature_descriptor = FeatureDescriptor(json_params['feature_descriptor'])
        self._depth_to_3d_sparse = calib.DepthTo3dSparse()
        self._executer = None
        self._keypoints_to_mat = tod_training.KeypointsToMat()
        self._camera_to_world = tod_training.CameraToWorld()
        self._model_stacker = tod_training.TodModelStacker()
        self._point_merger = tod_training.PointMerger()
        self._prepare_for_g2o = tod_training.PrepareForG2O(search_json_params=dict_to_cpp_json_str(json_params['search']))
        self._rescale_depth = capture.RescaledRegisteredDepth() #this is for SXGA mode scale handling.
        self._keypoint_validator = conversion.KeypointsValidator()
        self._g2o = SBA()

        self._image_duplicator = ecto.Passthrough()
        self._mask_duplicator = ecto.Passthrough()
        self._K_duplicator = ecto.Passthrough()
        self._R_duplicator = ecto.Passthrough()
        self._T_duplicator = ecto.Passthrough()
        self._depth_duplicator = ecto.Passthrough()

    def expose_inputs(self):
        return {}

    def expose_outputs(self):
        return {'points': self._point_merger['points'],
                'descriptors': self._point_merger['descriptors']}

    def expose_parameters(self):
        return {}

    def connections(self):
        observation_dealer = ecto.Dealer(typer=self._input_cell.inputs.at('observation'), iterable=self._obs_ids)
        sub_connections = [ observation_dealer[:] >> self._input_cell['observation'] ]

        # connect the input
        sub_connections += [ self._input_cell['image'] >> self._image_duplicator['in'],
                           self._input_cell['mask'] >> self._mask_duplicator['in'],
                           self._input_cell['depth'] >> self._depth_duplicator['in'],
                           self._input_cell['K'] >> self._K_duplicator['in'],
                           self._input_cell['R'] >> self._R_duplicator['in'],
                           self._input_cell['T'] >> self._T_duplicator['in'] ]

        sub_connections += [self._image_duplicator[:] >> self._feature_descriptor['image'],
                           self._image_duplicator[:] >> self._rescale_depth['image'],
                           self._mask_duplicator[:] >> self._feature_descriptor['mask'],
                           self._depth_duplicator[:] >> self._rescale_depth['depth'],
                           self._K_duplicator[:] >> self._depth_to_3d_sparse['K']]
        # Make sure the keypoints are in the mask and with a valid depth
        sub_connections += [self._feature_descriptor['keypoints', 'descriptors'] >>
                            self._keypoint_validator['keypoints', 'descriptors'],
                            self._K_duplicator[:] >> self._keypoint_validator['K'],
                            self._mask_duplicator[:] >> self._keypoint_validator['mask'],
                            self._rescale_depth['depth'] >> self._keypoint_validator['depth'] ]
        # transform the keypoints/depth into 3d points
        sub_connections += [ self._keypoint_validator['points'] >> self._depth_to_3d_sparse['points'],
                        self._rescale_depth['depth'] >> self._depth_to_3d_sparse['depth'],
                        self._R_duplicator[:] >> self._camera_to_world['R'],
                        self._T_duplicator[:] >> self._camera_to_world['T'],
                        self._depth_to_3d_sparse['points3d'] >> self._camera_to_world['points']]
        # store all the info
        sub_connections += [ self._camera_to_world['points'] >> self._model_stacker['points3d'],
                        self._keypoint_validator['points'] >> self._model_stacker['points'],
                        self._keypoint_validator['descriptors'] >> self._model_stacker['descriptors'],
                        self._K_duplicator[:] >> self._model_stacker['K'],
                        self._R_duplicator[:] >> self._model_stacker['R'],
                        self._T_duplicator[:] >> self._model_stacker['T'] ]

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
        self._executer = Executer(plasm=plasm, niter=0, outputs={'points3d':self._model_stacker,
                            'points':self._model_stacker, 'descriptors':self._model_stacker, 'K': self._model_stacker,
                            'quaternions': self._model_stacker, 'Ts': self._model_stacker})

        connections = [ self._executer['K', 'quaternions', 'Ts'] >> self._g2o['K', 'quaternions', 'Ts'],
                       self._executer['points3d', 'points', 'descriptors'] >>
                       self._prepare_for_g2o['points3d', 'points', 'descriptors'],
                       self._prepare_for_g2o['x', 'y', 'disparity', 'points'] >> self._g2o['x', 'y', 'disparity', 'points'],
                       self._g2o['points'] >> self._point_merger['points'],
                       self._prepare_for_g2o['ids'] >> self._point_merger['ids'],
                       self._executer['descriptors'] >> self._point_merger['descriptors'] ]

        return connections
