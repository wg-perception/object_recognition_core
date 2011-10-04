#!/usr/bin/env python
"""
Module defining the TOD trainer to train the TOD models
"""

from ecto_object_recognition import capture, tod_training
from ecto_opencv import calib, features2d, highgui
from g2o import SBA
from feature_descriptor import FeatureDescriptor
import ecto
import ecto_X

########################################################################################################################

class Trainer(ecto.BlackBox):
    """
    Depth can be of a different size, it will be resized to fit image and mask
    """
    feature_descriptor = FeatureDescriptor
    point_merger = tod_training.PointMerger
    prepare_for_g2o = tod_training.PrepareForG2O
    g2o = SBA
    observation_passthrough = ecto.Passthrough
    image_duplicator = ecto.Passthrough
    mask_duplicator = ecto.Passthrough
    K_duplicator = ecto.Passthrough
    R_duplicator = ecto.Passthrough
    T_duplicator = ecto.Passthrough
    depth_duplicator = ecto.Passthrough

    def __init__(self, **kwargs):
        self.source_plasm = kwargs.pop('source_plasm')
        self.source = kwargs.pop('source')
        ecto.BlackBox.__init__(self, **kwargs)

    def declare_params(self, p):
        p.declare('display', 'If true, displays images at runtime', False)
        p.forward('json_feature_descriptor_params', cell_name = 'feature_descriptor', cell_key = 'json_params')
        p.forward('json_search_params', cell_name = 'prepare_for_g2o', cell_key = 'search_json_params')

    def declare_io(self, p, i, o):
        o.forward('points', cell_name='point_merger', cell_key='points')
        o.forward('descriptors', cell_name='point_merger', cell_key='descriptors')

    def configure(self, p, i, o):
        self.feature_descriptor = self.feature_descriptor()
        self.depth_to_3d_sparse = calib.DepthTo3dSparse()
        self.keypoints_to_mat = tod_training.KeypointsToMat()
        self.camera_to_world = tod_training.CameraToWorld()
        self.model_stacker = tod_training.TodModelStacker()
        self.point_merger = self.point_merger()
        self.prepare_for_g2o = self.prepare_for_g2o(search_json_params=p.json_search_params)
        self.rescale_depth = capture.RescaledRegisteredDepth() #this is for SXGA mode scale handling.
        self.keypoint_validator = tod_training.KeypointsValidator()
        self.g2o = self.g2o()
        self.observation_passthrough = self.observation_passthrough()
        self.image_duplicator = self.image_duplicator()
        self.mask_duplicator = self.mask_duplicator()
        self.K_duplicator = self.K_duplicator()
        self.R_duplicator = self.R_duplicator()
        self.T_duplicator = self.T_duplicator()
        self.depth_duplicator = self.depth_duplicator()
        
        self._display = p.display

    def connections(self):
        sub_connections = [ self.source['image'] >> self.image_duplicator['in'],
                           self.source['mask'] >> self.mask_duplicator['in'],
                           self.source['depth'] >> self.depth_duplicator['in'],
                           self.source['K'] >> self.K_duplicator['in'],
                           self.source['R'] >> self.R_duplicator['in'],
                           self.source['T'] >> self.T_duplicator['in'] ]

        # connect the input
        sub_connections += [self.image_duplicator[:] >> self.feature_descriptor['image'],
                           self.image_duplicator[:] >> self.rescale_depth['image'],
                           self.mask_duplicator[:] >> self.feature_descriptor['mask'],
                           self.depth_duplicator[:] >> self.rescale_depth['depth'],
                           self.K_duplicator[:] >> self.depth_to_3d_sparse['K']]
        # Make sure the keypoints are in the mask and with a valid depth
        sub_connections += [self.feature_descriptor['keypoints', 'descriptors'] >>
                            self.keypoint_validator['keypoints', 'descriptors'],
                            self.K_duplicator[:] >> self.keypoint_validator['K'],
                            self.mask_duplicator[:] >> self.keypoint_validator['mask'],
                            self.rescale_depth['depth'] >> self.keypoint_validator['depth'] ]
        # transform the keypoints/depth into 3d points
        sub_connections += [ self.keypoint_validator['points'] >> self.depth_to_3d_sparse['points'],
                        self.rescale_depth['depth'] >> self.depth_to_3d_sparse['depth'],
                        self.R_duplicator[:] >> self.camera_to_world['R'],
                        self.T_duplicator[:] >> self.camera_to_world['T'],
                        self.depth_to_3d_sparse['points3d'] >> self.camera_to_world['points']]
        # store all the info
        sub_connections += [ self.camera_to_world['points'] >> self.model_stacker['points3d'],
                        self.keypoint_validator['points'] >> self.model_stacker['points'],
                        self.keypoint_validator['descriptors'] >> self.model_stacker['descriptors'],
                        self.K_duplicator[:] >> self.model_stacker['K'],
                        self.R_duplicator[:] >> self.model_stacker['R'],
                        self.T_duplicator[:] >> self.model_stacker['T'] ]

        if self._display:
            mask_view = highgui.imshow(name="mask")
            depth_view = highgui.imshow(name="depth");

            sub_connections += [ self.mask_duplicator[:] >> mask_view['image'],
                            self.depth_duplicator[:] >> depth_view['image']]
            # draw the keypoints
            keypoints_view = highgui.imshow(name="Keypoints")
            draw_keypoints = features2d.DrawKeypoints()
            sub_connections += [ self.image_duplicator[:] >> draw_keypoints['image'],
                            self.feature_descriptor['keypoints'] >> draw_keypoints['keypoints'],
                            draw_keypoints['image'] >> keypoints_view['image']]

        self.source_plasm.connect(sub_connections)
        executer = ecto_X.Executer(plasm=self.source_plasm, niter=0, outputs={'points3d':self.model_stacker,
                            'points':self.model_stacker, 'descriptors':self.model_stacker, 'K': self.model_stacker,
                            'quaternions': self.model_stacker, 'Ts': self.model_stacker})

        connections = [ executer['K', 'quaternions', 'Ts'] >> self.g2o['K', 'quaternions', 'Ts'],
                       executer['points3d', 'points', 'descriptors'] >>
                       self.prepare_for_g2o['points3d', 'points', 'descriptors'],
                       self.prepare_for_g2o['x', 'y', 'disparity', 'points'] >> self.g2o['x', 'y', 'disparity', 'points'],
                       self.g2o['points'] >> self.point_merger['points'],
                       self.prepare_for_g2o['ids'] >> self.point_merger['ids'],
                       executer['descriptors'] >> self.point_merger['descriptors'] ]

        return connections
