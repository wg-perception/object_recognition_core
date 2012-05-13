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
from object_recognition_core.db import tools as dbtools, models
from object_recognition_core.dbcells import ObservationReader

db_url = dbtools.DEFAULT_SERVER_URL

#database ritual
couch = couchdb.Server(db_url)
dbs = dbtools.init_object_databases(couch)
sessions = dbs['sessions']
observations = dbs['observations']
results = models.Session.all(sessions)
obs_ids = []

for session in results:
    obs_ids += models.find_all_observations_for_session(observations, session.id)

if len(obs_ids) == 0:
    raise RuntimeError("There are no observations available.")

db_reader = capture.ObservationReader('db_reader', db_params=ObjectDbParameters({'type':'CouchDB', 'root':db_url,
                                                                                 'collection':'observations'}))
#observation dealer will deal out each observation id.
observation_dealer = ecto.Dealer(tendril=db_reader.inputs.at('observation'), iterable=obs_ids)
fps = highgui.FPSDrawer('FPS drawer')
plasm = ecto.Plasm()
#View all of the observations.
plasm.connect(
    observation_dealer[:] >> db_reader['observation'],
    db_reader['image'] >> fps[:],
    fps[:] >> highgui.imshow('image display', name='image')[:],
    db_reader['depth'] >> highgui.imshow('depth display', name='depth')[:],
    db_reader['mask'] >> highgui.imshow('mask display', name='mask')[:],
)
from ecto.opts import doit
doit(plasm, "View observations from the database.", locals=vars())
