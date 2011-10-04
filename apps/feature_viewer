#!/usr/bin/env python
"""
module that displays a TOD model for a given object_id
"""
import sys
import argparse
import time
import tempfile
import os
import math
import subprocess

import couchdb

import ecto
from ecto_opencv import highgui
import object_recognition
from object_recognition import dbtools, models, capture
from object_recognition.common.utils.parser import ObjectRecognitionParser
from ecto_object_recognition import tod_detection, conversion
import ecto_pcl

if __name__ == '__main__':
    parser = ObjectRecognitionParser()
    dbtools.add_db_arguments(parser)
    parser.add_argument('--object_id', help='The id of the object for which the TOD model will be displayed.')

    # read the object_ids
    args = parser.parse_args()
    if hasattr(args, 'object_id') and args.object_id:
        object_id = args.object_id
    else:
        object_id = "e2449bdc43fd6d9dd646fcbcd00d8197"

    #database ritual
    db_url = args.db_root
    db_json_params = '{"root": "%s", "type": "CouchDB"}' % args.db_root
    couch = couchdb.Server(db_url)
    dbs = dbtools.init_object_databases(couch)

    tod_models = models.find_model_for_object(dbs, object_id, 'TOD')

    if len(tod_models) < 1:
        raise RuntimeError("There are no tod models available.")

    db_reader = tod_detection.ModelReader('db_reader', db_json_params=db_json_params, collection='object_recognition')
    #observation dealer will deal out each observation id.
    observation_dealer = ecto.Dealer(typer=db_reader.inputs.at('model_id'), iterable=tod_models)
    to_pcl = conversion.MatToPointCloudXYZ()
    pcl_cloud = ecto_pcl.PointCloudT2PointCloud(format=ecto_pcl.Format.XYZ)
    cloud_viewer = ecto_pcl.CloudViewer()
    plasm = ecto.Plasm()
    #View all of the observations.
    plasm.connect(
        observation_dealer[:] >> db_reader['model_id'],
        db_reader['points'] >> to_pcl['points'],
        to_pcl['point_cloud'] >> pcl_cloud[:],
        pcl_cloud[:] >> cloud_viewer[:]
    )
    
    from ecto.opts import doit
    doit(plasm, "View observations from the database.", locals=vars())
    
    raw_input()
