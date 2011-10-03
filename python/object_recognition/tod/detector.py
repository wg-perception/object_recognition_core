#!/usr/bin/env python
"""
Module defining the TOD detector to find objects in a scene
"""

import ecto
from feature_descriptor import FeatureDescriptor
from ecto_object_recognition import tod_detection
from ecto_opencv import features2d, highgui
from object_recognition.common.utils import json_helper

########################################################################################################################

class TodDetector(ecto.BlackBox):
    feature_descriptor = FeatureDescriptor
    descriptor_matcher = tod_detection.DescriptorMatcher
    guess_generator = tod_detection.GuessGenerator
    image_duplicator = ecto.Passthrough

    def __init__(self, **kwargs):
        self._db_params = kwargs.pop('db_params')
        self._tod_params = kwargs.pop('feature_descriptor_params')
        self._guess_params = kwargs.pop('guess_params')
        self._search_params = kwargs.pop('search_params')
        self._object_ids = kwargs.pop('object_ids')
        self._display = kwargs.pop('display')

        ecto.BlackBox.__init__(self, **kwargs)

    def declare_params(self, p):
        pass

    def declare_io(self, p, i, o):
        i.forward('image', cell_name = 'image_duplicator', cell_key = 'in')
        i.forward('mask', cell_name = 'feature_descriptor', cell_key = 'mask')
        i.forward('points3d', cell_name = 'guess_generator', cell_key = 'points3d')

        o.forward('object_ids', cell_name = 'guess_generator', cell_key = 'object_ids')
        o.forward('Rs', cell_name = 'guess_generator', cell_key = 'Rs')
        o.forward('Ts', cell_name = 'guess_generator', cell_key = 'Ts')
        o.forward('keypoints', cell_name = 'feature_descriptor', cell_key = 'keypoints')

    def configure(self, p, i, o):
        self.feature_descriptor = FeatureDescriptor(json_helper.dict_to_cpp_json_str(self._tod_params))
        self.descriptor_matcher = tod_detection.DescriptorMatcher("Matcher",
                                db_json_params=json_helper.dict_to_cpp_json_str(self._db_params),
                                object_ids=self._object_ids,
                                search_json_params=json_helper.dict_to_cpp_json_str(self._search_params))
        self.guess_generator = tod_detection.GuessGenerator("Guess Gen",
                                                json_params=json_helper.dict_to_cpp_json_str(self._guess_params))

        self.image_duplicator = ecto.Passthrough()

    def connections(self):
        # make sure the inputs reach the right cells
        connections = [self.image_duplicator[:] >> self.feature_descriptor['image'],
                       self.image_duplicator[:] >> self.guess_generator['image'],]

        connections += [self.feature_descriptor['keypoints'] >> self.guess_generator['keypoints'],
                self.feature_descriptor['descriptors'] >> self.descriptor_matcher['descriptors'],
                self.descriptor_matcher['matches', 'matches_3d', 'spans', 'id_correspondences'] >>
                self.guess_generator['matches', 'matches_3d', 'spans', 'id_correspondences']]

        if self._display:
            # display the found keypoints
            image_view = highgui.imshow(name="RGB")
            keypoints_view = highgui.imshow(name="Keypoints")
            draw_keypoints = features2d.DrawKeypoints()

            connections += [ self.image_duplicator[:] >> image_view['image'],
                           self.image_duplicator[:] >> draw_keypoints['image'],
                           self.feature_descriptor['keypoints'] >> draw_keypoints['keypoints'],
                           draw_keypoints['image'] >> keypoints_view['image']
                           ]

        return connections
