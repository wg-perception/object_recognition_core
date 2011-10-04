#!/usr/bin/env python

import ecto
from object_recognition.tod.detector import TodDetector

T = TodDetector(collection = '', db_params={'type':'CouchDB', 'root': 'bogus'}, feature_descriptor_params='',
                                   guess_params='', search_params='',
                                   display=True)
print T.__doc__
