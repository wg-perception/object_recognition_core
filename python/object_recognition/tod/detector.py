#!/usr/bin/env python
"""
Module defining the TOD detector to find objects in a scene
"""

import couchdb
import ecto
from feature_descriptor import FeatureDescriptor
from ecto_object_recognition import tod_detection
from ecto_opencv import features2d, highgui
from object_recognition.common.utils import json_helper
from object_recognition import dbtools, models

########################################################################################################################

class TodDetector(ecto.BlackBox):
    feature_descriptor = FeatureDescriptor
    descriptor_matcher = tod_detection.DescriptorMatcher
    guess_generator = tod_detection.GuessGenerator
    image_duplicator = ecto.Passthrough

    def __init__(self, db_params, collection, feature_descriptor_params, guess_params, search_params, object_ids, display = False,
                 **kwargs):
        self._db_params = db_params
        self._collection = collection
        self._tod_params = feature_descriptor_params
        self._guess_params = guess_params
        self._search_params = search_params
        self._object_ids = object_ids
        
        
        # initialize the DB
        if db_params['type'].lower() == 'couchdb':
            db = dbtools.init_object_databases(couchdb.Server(db_params['root']))

        self._model_ids = []
        for object_id in object_ids:
            for model_id in models.find_model_for_object(db, object_id, 'TOD'):
                self._model_ids.extend([model_id])

        self._display = display

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
                                collection_models = self._collection,
                                db_json_params=json_helper.dict_to_cpp_json_str(self._db_params),
                                model_ids=self._model_ids,
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
