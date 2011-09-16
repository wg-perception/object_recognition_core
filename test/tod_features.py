#!/usr/bin/env python

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
from ecto_object_recognition import tod_detection, conversion
import ecto_pcl
db_json_params = '''
    {
        "type": "CouchDB",
        "url": "http://localhost:5984"
    }
'''
db_url = dbtools.DEFAULT_SERVER_URL

#database ritual
couch = couchdb.Server(db_url)
dbs = dbtools.init_object_databases(couch)

object_id = "6279b8c2599f44103e0573e75cfef9da"

tod_models = models.find_tod_model_for_object(dbs['models'], object_id)
print tod_models
if len(tod_models) < 1:
    raise RuntimeError("There are no tod models available.")

db_reader = tod_detection.ModelReader('db_reader', db_json_params=db_json_params, collection='models')
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
