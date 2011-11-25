#!/usr/bin/env python
"""
Module defining the TOD trainer to train the TOD models
"""

from ecto_object_recognition import capture, tod_training
from ecto_opencv import calib, features2d, highgui
from g2o import SbaDisparity
from feature_descriptor import FeatureDescriptor
import ecto
from object_recognition.pipelines.training import TrainingPipeline
from object_recognition.common.utils import dict_to_cpp_json_str

########################################################################################################################
class TODModelBuilder(ecto.BlackBox):
    """
    """
    def declare_params(self, p):
        self.feature_descriptor = FeatureDescriptor()
        p.declare('visualize', 'If true, displays images at runtime', False)
        p.forward('json_feature_descriptor_params', cell_name='feature_descriptor', cell_key='json_params')

    def declare_io(self, p, i, o):
        self.source = ecto.PassthroughN(items=dict(image='An image',
                                                   depth='A depth image',
                                                   mask='A mask for valid object pixels.',
                                                   K='The camera matrix',
                                                   R='The rotation matrix',
                                                   T='The translation vector',
                                                   frame_number='The frame number.'
                                                   )
                                        )
        self.model_stacker = tod_training.TodModelStacker()

        i.forward_all('source')
        o.forward_all('model_stacker')

    def configure(self, p, i, o):
        self.depth_to_3d_sparse = calib.DepthTo3dSparse()
        self.keypoints_to_mat = features2d.KeypointsToMat()
        self.camera_to_world = tod_training.CameraToWorld()
        self.model_stacker = tod_training.TodModelStacker()
        self.rescale_depth = capture.RescaledRegisteredDepth() #this is for SXGA mode scale handling.
        self.keypoint_validator = tod_training.KeypointsValidator()
        self.visualize = p.visualize

    def connections(self):
        graph = []
        # connect the input
        graph += [self.source['image'] >> self.feature_descriptor['image'],
                           self.source['image'] >> self.rescale_depth['image'],
                           self.source['mask'] >> self.feature_descriptor['mask'],
                           self.source['depth'] >> self.rescale_depth['depth'],
                           self.source['K'] >> self.depth_to_3d_sparse['K']]

        # Make sure the keypoints are in the mask and with a valid depth
        graph += [self.feature_descriptor['keypoints', 'descriptors'] >>
                            self.keypoint_validator['keypoints', 'descriptors'],
                            self.source['K'] >> self.keypoint_validator['K'],
                            self.source['mask'] >> self.keypoint_validator['mask'],
                            self.rescale_depth['depth'] >> self.keypoint_validator['depth'] ]

        # transform the keypoints/depth into 3d points
        graph += [ self.keypoint_validator['points'] >> self.depth_to_3d_sparse['points'],
                        self.rescale_depth['depth'] >> self.depth_to_3d_sparse['depth'],
                        self.source['R'] >> self.camera_to_world['R'],
                        self.source['T'] >> self.camera_to_world['T'],
                        self.depth_to_3d_sparse['points3d'] >> self.camera_to_world['points']]

        # store all the info
        graph += [ self.camera_to_world['points'] >> self.model_stacker['points3d'],
                        self.keypoint_validator['points'] >> self.model_stacker['points'],
                        self.keypoint_validator['descriptors'] >> self.model_stacker['descriptors'],
                        self.source['K', 'R', 'T'] >> self.model_stacker['K', 'R', 'T'],
                        ]

        if self.visualize:
            mask_view = highgui.imshow(name="mask")
            depth_view = highgui.imshow(name="depth")

            graph += [ self.source['mask'] >> mask_view['image'],
                       self.source['depth'] >> depth_view['image']]
            # draw the keypoints
            keypoints_view = highgui.imshow(name="Keypoints")
            draw_keypoints = features2d.DrawKeypoints()
            graph += [ self.source['image'] >> draw_keypoints['image'],
                       self.feature_descriptor['keypoints'] >> draw_keypoints['keypoints'],
                       draw_keypoints['image'] >> keypoints_view['image']]

        return graph


class TODPostProcessor(ecto.BlackBox):
    """
    """
    prepare_for_g2o = tod_training.PrepareForG2O
    g2o = SbaDisparity
    point_merger = tod_training.PointMerger
    model_filler = tod_training.ModelFiller

    def declare_params(self, p):
        p.forward('json_search_params', cell_name='prepare_for_g2o', cell_key='search_json_params')

    def declare_io(self, p, i, o):
        self.source = ecto.PassthroughN(items=dict(K='The camera matrix',
                                                   quaternions='A vector of quaternions',
                                                   Ts='A vector of translation vectors',
                                                   descriptors='A stacked vector of descriptors',
                                                   points='The 2D measurements per point.',
                                                   points3d='The estimated 3d position of the points (3-channel matrices).'
                                                   )
                                        )
        i.forward_all('source')
        o.forward_all('model_filler')

    def configure(self, p, i, o):
        self.point_merger = self.point_merger()
        self.prepare_for_g2o = self.prepare_for_g2o(search_json_params=p.json_search_params)
        self.g2o = self.g2o()
        self.model_filler = self.model_filler()

    def connections(self):
        graph = [self.source['points', 'points3d', 'descriptors'] >> self.prepare_for_g2o['points', 'points3d', 'descriptors'],
                 self.source['Ts', 'quaternions', 'K'] >> self.g2o['Ts', 'quaternions', 'K'],
                 self.source['descriptors'] >> self.point_merger['descriptors'],
                ]
        graph += [
                 self.prepare_for_g2o['x', 'y', 'disparity', 'points'] >> self.g2o['x', 'y', 'disparity', 'points'],
                 self.g2o['points'] >> self.point_merger['points'],
                 self.prepare_for_g2o['ids'] >> self.point_merger['ids'],
                 self.point_merger['points', 'descriptors'] >> self.model_filler['points', 'descriptors']
                ]
        return graph

class TODTrainingPipeline(TrainingPipeline):
    '''Implements the training pipeline functions'''

    @classmethod
    def type_name(cls):
        return "TOD"

    def incremental_model_builder(self, pipeline_params, args):
        feature_params = pipeline_params.get("feature_descriptor", False)
        if not feature_params:
            raise RuntimeError("You must supply feature_descriptor parameters for TOD.")
        #grab visualize if works.
        visualize = getattr(args, 'visualize', False)
        return TODModelBuilder(json_feature_descriptor_params=dict_to_cpp_json_str(feature_params), visualize=visualize)

    def post_processor(self, pipeline_params, _args):
        search_params = pipeline_params.get("search", False)
        if not search_params:
            raise RuntimeError("You must supply search parameters for TOD.")
        return TODPostProcessor(json_search_params=dict_to_cpp_json_str(search_params))
