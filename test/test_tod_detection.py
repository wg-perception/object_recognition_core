#!/usr/bin/env python

from ecto_object_recognition.object_recognition_db import DbModels, ObjectDbParameters
from object_recognition.tod.detector import TodDetectionPipeline
import ecto

T = TodDetectionPipeline()

submethod = {'descriptor': {'type':'ORB'}}
parameters = {'feature': {'type': 'ORB'}, 'search': {}, 'guess':{}, 'db': {'type': 'CouchDB', 'root': 'http://localhost:5984'}}

T.detector(submethod=submethod, parameters=parameters, object_ids=[])
