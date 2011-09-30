#!/usr/bin/env python
import roscompat
import os
import string
import sys
import time
import couchdb
from object_recognition import dbtools, models
from ecto_object_recognition import capture, tod_training
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, features2d
from object_recognition.tod.feature_descriptor import FeatureDescriptor
from object_recognition.tod.trainer import Trainer as TodTrainer
from object_recognition.common.utils.parser import ObjectRecognitionParser
import ecto
from object_recognition.common.utils import json_helper
from object_recognition.dbtools import add_db_arguments, args_to_dict

DEBUG = False
DISPLAY = True

########################################################################################################################

if __name__ == '__main__':

    parser = ObjectRecognitionParser()
    add_db_arguments(parser)
    
    parser.add_argument('--tod_combination', dest='tod_combination',
                        help='The combo')
    parser.add_argument('--tod_n_features', dest='tod_n_features', type=int,
                        help='The combo')
    parser.add_argument('object_ids', nargs='+',
                        help='The combo')

    args = parser.parse_args()

    db_url = args.db_root
    db_dict = args_to_dict(args)

    tod_params = {'feature_descriptor': {'combination': args.tod_combination, 'feature':{'n_features': args.tod_n_features}},
                  'search': {"type": "LSH",
            "n_tables": 16,
            "key_size": 18,
            "multi_probe_level": 2,
            "ratio": 0.8,
            "radius": 50
                } }

    # initialize the DB
    couch = couchdb.Server(db_url)
    db = dbtools.init_object_databases(couch)

    for object_id in args.object_ids:
        object_id = object_id.encode('ascii')
        db_reader = capture.ObservationReader("db_reader", db_url=db_url)
        obs_ids = models.find_all_observations_for_object(db, object_id)
        if not obs_ids:
            print 'no observations found for object %s' % object_id
            continue

        # connect to the model computation
        plasm = ecto.Plasm()
        tod_model = TodTrainer(plasm, db_reader, obs_ids, tod_params, DISPLAY)

        # persist to the DB
        db_writer = tod_training.ModelInserter("db_writer", collection_models=args.db_collection,
                                    db_json_params=json_helper.dict_to_cpp_json_str(db_dict), object_id=object_id,
                                    model_json_params=json_helper.dict_to_cpp_json_str(tod_params))
        orb_params = None
        # TODO
        #db_writer.add_misc(orb_params)

        plasm.connect(tod_model['points', 'descriptors'] >> db_writer['points', 'descriptors'])

        if DEBUG:
            #render the DAG with dot
            print plasm.viz()
            ecto.view_plasm(plasm)

        sched = ecto.schedulers.Singlethreaded(plasm)
        sched.execute(niter=1)
