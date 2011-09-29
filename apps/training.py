#!/usr/bin/env python
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
from argparse import ArgumentParser
import ecto
from object_recognition.common.utils import json_helper

DEBUG = False
DISPLAY = True

########################################################################################################################

def parse_options():
    parser = ArgumentParser()
    parser.add_argument("-c", "--config_file", dest="config_file",
                      help='the file containing the configuration as JSON. It should contain the following fields.\n'
                      '"feature_descriptor": with parameters for "combination", "feature" and "descriptor".\n'
                      '"db": parameters about the db: "type", "url".\n'
                      '"objects_ids": the list of object to process, e.g. ["amys_country_cheddar_bowl",'
                      '"band_aid_plastic_strips"]\n'
                      )

    return parser.parse_args()

########################################################################################################################

if __name__ == '__main__':

    args = parse_options()

    # define the input
    if args.config_file is None or not os.path.exists(args.config_file):
        raise 'option file does not exist'

    json_params = json_helper.file_to_json(args.config_file)
    db_url = str(json_params['db']['url'])

    # initialize the DB
    couch = couchdb.Server(db_url)
    dbs = dbtools.init_object_databases(couch)

    object_ids = json_params['object_ids']
    for object_id in object_ids:
        object_id = object_id.encode('ascii')
        db_reader = capture.ObservationReader("db_reader", db_url=db_url)
        obs_ids = models.find_all_observations_for_object(dbs['observations'], object_id)

        # connect to the model computation
        plasm = ecto.Plasm()
        tod_model = TodTrainer(plasm, db_reader, obs_ids[:3], json_params['tod'], DISPLAY)

        # persist to the DB
        db_writer = tod_training.ModelInserter("db_writer", collection_models='models',
                                    db_json_params=json_helper.dict_to_cpp_json_str(json_params['db']), object_id=object_id,
                                    model_json_params=json_helper.dict_to_cpp_json_str(json_params['tod']))
        orb_params = None
        # TODO
        #db_writer.add_misc(orb_params)
        
        # never execute the db_writer
        plasm.connect(tod_model['points', 'descriptors'] >> db_writer['points', 'descriptors'])

        if DEBUG:
            #render the DAG with dot
            print plasm.viz()
            ecto.view_plasm(plasm)

        sched = ecto.schedulers.Singlethreaded(plasm)
        sched.execute(niter=1)
