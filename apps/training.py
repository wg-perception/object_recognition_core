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

    parser.add_argument('-c', help='Config file')
    parser.add_argument('--object_ids', help='If set, it overrides the list of object_ids in the config file')

    args = parser.parse_args()

    params = yaml.load(open(args.c))

    # read the object_ids
    if hasattr(args, 'object_ids') and args.object_ids:
        object_ids = args.object_ids[1:-1].split(',')
    else:
        object_ids = params['object_ids']

    # read some parameters
    db_dict = params['db']
    db_url = db_dict['root']

    pipeline_params = []
    for key , value in params.iteritems():
        if key.startswith('pipeline'):
            pipeline_params.append(value)

    # initialize the DB
    if db_dict['type'].lower() == 'couchdb':
        db = dbtools.init_object_databases(couchdb.Server(db_url))

    for object_id in object_ids:
        object_id = object_id.encode('ascii')
        db_reader = capture.ObservationReader("db_reader", db_url=db_url)
        obs_ids = models.find_all_observations_for_object(db, object_id)
        if not obs_ids:
            print 'no observations found for object %s' % object_id
            continue

        # define the input and connect to it
        source_plasm = ecto.Plasm()
        observation_dealer = ecto.Dealer(typer=db_reader.inputs.at('observation'), iterable=obs_ids)
        source_plasm.connect(observation_dealer[:] >> db_reader['observation'])
        
        main_plasm = ecto.Plasm()
        # connect to the model computation
        for pipeline_param in pipeline_params:
            #define the trainer
            if pipeline_param['type'] == 'TOD':
                trainer = TodTrainer(json_search_params=json_helper.dict_to_cpp_json_str(pipeline_param['search']),
                                     json_feature_descriptor_params=json_helper.dict_to_cpp_json_str(pipeline_param['feature_descriptor']),
                                     display=DISPLAY, source=db_reader, source_plasm=source_plasm)

            # define the output
            db_writer = tod_training.ModelInserter("db_writer", collection_models=db_dict['collection'],
                                        db_json_params=json_helper.dict_to_cpp_json_str(db_dict), object_id=object_id,
                                        model_json_params=json_helper.dict_to_cpp_json_str(pipeline_param))
            orb_params = None
            # TODO
            #db_writer.add_misc(pipeline_param)

            # connect the output
            main_plasm.connect(trainer['points', 'descriptors'] >> db_writer['points', 'descriptors'])
    
        if DEBUG:
            #render the DAG with dot
            print main_plasm.viz()
            ecto.view_plasm(main_plasm)
    
        sched = ecto.schedulers.Singlethreaded(main_plasm)
        sched.execute(niter=1)
