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
    def __init__(self, plasm, db_json_params, tod_json_params, object_ids, display = False):
        ecto.BlackBox.__init__(self, plasm)

        self._object_ids = object_ids
        self._display = display

        # parse the JSON and load the appropriate feature descriptor module
        self.feature_descriptor = FeatureDescriptor(tod_json_params['feature_descriptor'])
        self.descriptor_matcher = tod_detection.DescriptorMatcher("Matcher",
                                db_json_params=json_helper.dict_to_cpp_json_str(db_json_params), object_ids=object_ids,
                                search_json_params=json_helper.dict_to_cpp_json_str(tod_json_params['search']))
        self.guess_generator = tod_detection.GuessGenerator("Guess Gen",
                                                json_params=json_helper.dict_to_cpp_json_str(tod_json_params['guess']))

        self._image_duplicator = ecto.Passthrough()

    def expose_inputs(self):
        return {'image': self._image_duplicator['in'],
                'mask':self.feature_descriptor['mask'],
                'points3d':self.guess_generator['points3d']}

    def expose_outputs(self):
        return {'object_ids': self.guess_generator['object_ids'],
                'Rs': self.guess_generator['Rs'],
                'Ts': self.guess_generator['Ts'],
                'keypoints': self.feature_descriptor['keypoints']}

    def expose_parameters(self):
        return {}

    def connections(self):
        # make sure the inputs reach the right cells
        connections = [self._image_duplicator[:] >> self.feature_descriptor['image']]

        connections += [self.feature_descriptor['keypoints'] >> self.guess_generator['keypoints'],
                self.feature_descriptor['descriptors'] >> self.descriptor_matcher['descriptors'],
                self.descriptor_matcher['matches', 'matches_3d', 'spans', 'id_correspondences'] >>
                self.guess_generator['matches', 'matches_3d', 'spans', 'id_correspondences']]

        if self._display:
            # display the found keypoints
            image_view = highgui.imshow(name="RGB")
            keypoints_view = highgui.imshow(name="Keypoints")
            draw_keypoints = features2d.DrawKeypoints()

            connections += [ self._image_duplicator[:] >> image_view['image'],
                           self._image_duplicator[:] >> draw_keypoints['image'],
                           self.feature_descriptor['keypoints'] >> draw_keypoints['keypoints'],
                           draw_keypoints['image'] >> keypoints_view['image']
                           ]

        return connections
