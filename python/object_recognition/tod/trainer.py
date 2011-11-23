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

########################################################################################################################
class TODModelBuilder(ecto.BlackBox):
    """
    """
    feature_descriptor = FeatureDescriptor

    def declare_params(self, p):
        p.declare('display', 'If true, displays images at runtime', False)
        p.forward('json_feature_descriptor_params', cell_name='feature_descriptor', cell_key='json_params')

    def declare_io(self, p, i, o):
        self.source = ecto.PassthroughN(what=dict(image='An image',
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
        self.feature_descriptor = self.feature_descriptor()
        self.depth_to_3d_sparse = calib.DepthTo3dSparse()
        self.keypoints_to_mat = features2d.KeypointsToMat()
        self.camera_to_world = tod_training.CameraToWorld()
        self.model_stacker = tod_training.TodModelStacker()
        self.rescale_depth = capture.RescaledRegisteredDepth() #this is for SXGA mode scale handling.
        self.keypoint_validator = tod_training.KeypointsValidator()
        self._display = p.display

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

        if self._display:
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
        i.forward_all('g2o')
        i.forward_all('prepare_for_g2o')
        i.forward('descriptors', 'point_merger')
        o.forward_all('model_filler')

    def configure(self, p, i, o):
        self.point_merger = self.point_merger()
        self.prepare_for_g2o = self.prepare_for_g2o(search_json_params=p.json_search_params)
        self.g2o = self.g2o()

    def connections(self):
        graph = [
                 self.prepare_for_g2o['x', 'y', 'disparity', 'points'] >> self.g2o['x', 'y', 'disparity', 'points'],
                 self.g2o['points'] >> self.point_merger['points'],
                 self.prepare_for_g2o['ids'] >> self.point_merger['ids'],
                 self.point_merger['points', 'descriptors'] >> model_filler['points', 'descriptors']
                ]
        return graph

class TODTrainingPipeline(TrainingPipeline):
    '''Implements the training pipeline functions'''

    def incremental_model_builder(self,pipeline_params):
        return TODModelBuilder

    def post_processor(self,pipeline_params):
        return TODPostProcessor
