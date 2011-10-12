#!/usr/bin/env python
"""
Module defining the TOD detector to find objects in a scene
"""

import couchdb
import ecto
from feature_descriptor import FeatureDescriptor
from ecto_object_recognition import tod_detection
from ecto_opencv import features2d, highgui, imgproc
from object_recognition.common.utils import json_helper
from object_recognition import dbtools, models
import ecto_ros, ecto_sensor_msgs
ImagePub = ecto_sensor_msgs.Publisher_Image

class TodDetectorLoader(ecto.BlackBox):
    """
    Blackbox that loads the descriptors from the db
    """
    tod_detection_loader = tod_detection.DescriptorLoader

    def declare_params(self, p):
        p.forward('object_ids', cell_name='tod_detection_loader', cell_key='object_ids',
                  doc='The list of objects to load the models from.')
        p.forward('model_ids', cell_name='tod_detection_loader', cell_key='model_ids',
                  doc='The list of objects to load the models from.')
        p.forward('db_params', cell_name='tod_detection_loader', cell_key='db_params',
                  doc='The DB parameters.')
        p.forward('collection_models', cell_name='tod_detection_loader', cell_key='collection_models',
                  doc='The collection where the models are.')
        p.forward('feature_descriptor_params', cell_name='tod_detection_loader', cell_key='feature_descriptor_params',
                  doc='The parameters of the feature/descriptor, to find the right models (unused now).')

    def declare_io(self, _p, _i, o):
        o.forward('descriptors', cell_name='tod_detection_loader', cell_key='descriptors')
        o.forward('do_update', cell_name='tod_detection_loader', cell_key='do_update')
        o.forward('features3d', cell_name='tod_detection_loader', cell_key='features3d')
        o.forward('id_correspondences', cell_name='tod_detection_loader', cell_key='id_correspondences')
        o.forward('spans', cell_name='tod_detection_loader', cell_key='spans')

    def configure(self, p, _i, _o):
        pass

    def connections(self):
        return [ self.tod_detection_loader ]

########################################################################################################################

class TodDetector(ecto.BlackBox):
    feature_descriptor = FeatureDescriptor
    descriptor_matcher = tod_detection.DescriptorMatcher
    guess_generator = tod_detection.GuessGenerator
    image_duplicator = ecto.Passthrough
    message_cvt = ecto_ros.Mat2Image

    def __init__(self, feature_descriptor_params, guess_params, search_params, display=False, **kwargs):
        self._tod_params = feature_descriptor_params
        self._guess_params = guess_params
        self._search_params = search_params

        self._display = display

        ecto.BlackBox.__init__(self, **kwargs)

    def declare_params(self, p):
        p.forward('rgb_frame_id', cell_name='message_cvt', cell_key='frame_id')

    def declare_io(self, _p, i, o):
        i.forward('image', cell_name='image_duplicator', cell_key='in')
        i.forward('mask', cell_name='feature_descriptor', cell_key='mask')
        i.forward('points3d', cell_name='guess_generator', cell_key='points3d')
        i.forward('spans', cell_name='guess_generator', cell_key='spans')
        i.forward('id_correspondences', cell_name='guess_generator', cell_key='id_correspondences')
        i.forward('descriptors_db', cell_name='descriptor_matcher', cell_key='descriptors_db')
        i.forward('features3d_db', cell_name='descriptor_matcher', cell_key='features3d_db')
        i.forward('do_update', cell_name='descriptor_matcher', cell_key='do_update')

        o.forward('object_ids', cell_name='guess_generator', cell_key='object_ids')
        o.forward('Rs', cell_name='guess_generator', cell_key='Rs')
        o.forward('Ts', cell_name='guess_generator', cell_key='Ts')
        o.forward('keypoints', cell_name='feature_descriptor', cell_key='keypoints')

    def configure(self, _p, _i, _o):
        self.feature_descriptor = FeatureDescriptor(json_params=json_helper.dict_to_cpp_json_str(self._tod_params))
        self.descriptor_matcher = tod_detection.DescriptorMatcher("Matcher",
                                search_json_params=json_helper.dict_to_cpp_json_str(self._search_params))
        self.message_cvt = ecto_ros.Mat2Image()

        guess_params = {}
        for key in [ 'min_inliers', 'n_ransac_iterations', 'sensor_error']:
            if key in self._guess_params:
                guess_params[key] = self._guess_params[key]
        guess_params['do_display'] = self._display
        self.guess_generator = tod_detection.GuessGenerator("Guess Gen", **guess_params)

        self.image_duplicator = ecto.Passthrough()

    def connections(self):
        # make sure the inputs reach the right cells
        connections = [self.image_duplicator[:] >> self.feature_descriptor['image'],
                       self.image_duplicator[:] >> self.guess_generator['image'], ]

        connections += [ self.feature_descriptor['keypoints'] >> self.guess_generator['keypoints'],
                self.feature_descriptor['descriptors'] >> self.descriptor_matcher['descriptors'],
                self.descriptor_matcher['matches', 'matches_3d'] >> self.guess_generator['matches', 'matches_3d'] ]

        pub_features = ImagePub("Features Pub", topic_name='features')
        cvt_color = imgproc.cvtColor(flag=imgproc.RGB2GRAY)

        draw_keypoints = features2d.DrawKeypoints()
        connections += [   self.image_duplicator[:] >> cvt_color[:],
                           cvt_color[:] >> draw_keypoints['image'],
                           self.feature_descriptor['keypoints'] >> draw_keypoints['keypoints'],
                           draw_keypoints['image'] >> self.message_cvt[:],
                           self.message_cvt[:] >> pub_features[:],
                           ]

        if self._display:
            # display the found keypoints
            image_view = highgui.imshow(name="RGB")
            keypoints_view = highgui.imshow(name="Keypoints")

            connections += [ self.image_duplicator[:] >> image_view['image'],
                           draw_keypoints['image'] >> keypoints_view['image']
                           ]

        return connections
