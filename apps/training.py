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
import yaml

DEBUG = False
DISPLAY = True

########################################################################################################################

if __name__ == '__main__':

    parser = ObjectRecognitionParser()
    add_db_arguments(parser)

    parser.add_argument('-c', help='Config file')

    args = parser.parse_args()

    # read some parameters
    params = yaml.load(open(args.c))
    db_dict = params['db']
    db_url = db_dict['root']

    pipeline_params = []
    for key , value in params.iteritems():
        if key.startswith('pipeline'):
            pipeline_params.append(value)

    # initialize the DB
    if db_dict['type'].lower()=='couchdb':
        db = dbtools.init_object_databases(couchdb.Server(db_url))

    for object_id in params['object_ids']:
        object_id = object_id.encode('ascii')
        db_reader = capture.ObservationReader("db_reader", db_url=db_url)
        obs_ids = models.find_all_observations_for_object(db, object_id)
        if not obs_ids:
            print 'no observations found for object %s' % object_id
            continue

        # connect to the model computation
        for pipeline_param in pipeline_params:
            plasm = ecto.Plasm()
            trainer = TodTrainer(plasm, db_reader, obs_ids, pipeline_param, DISPLAY)
    
            # persist to the DB
            db_writer = tod_training.ModelInserter("db_writer", collection_models=args.db_collection,
                                        db_json_params=json_helper.dict_to_cpp_json_str(db_dict), object_id=object_id,
                                        model_json_params=json_helper.dict_to_cpp_json_str(pipeline_param))
            orb_params = None
            # TODO
            #db_writer.add_misc(pipeline_param)
    
            plasm.connect(trainer['points', 'descriptors'] >> db_writer['points', 'descriptors'])
    
            if DEBUG:
                #render the DAG with dot
                print plasm.viz()
                ecto.view_plasm(plasm)
    
            sched = ecto.schedulers.Singlethreaded(plasm)
            sched.execute(niter=1)
