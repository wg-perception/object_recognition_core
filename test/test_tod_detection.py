#!/usr/bin/env python

import ecto
from object_recognition.tod.detector import TodDetector
from ecto_object_recognition.object_recognition_db import ObjectDbParameters

T = TodDetector([],[],ObjectDbParameters({'type':'CouchDB', 'root': 'bogus'}), '', '{}',
                                   '{}', '{}',
                                   display=True)
print T.__doc__
