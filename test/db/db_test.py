#!/usr/bin/env python

from object_recognition_core.boost.interface import ObjectDb, ObjectDbParameters

db_params_raw = {'type': 'CouchDB', 'root': 'http://localhost:5984', 'collection': 'object_recognition'}

db_params = ObjectDbParameters(db_params_raw)

db = ObjectDb(db_params)
db_params_new = db.parameters().raw

for dic1, dic2 in [(db_params_raw, db_params_new), (db_params_new, db_params_raw)]:
    for k, v in dic1.items():
        if k in dic2 and dic2[k]!=v:
             raise RuntimeError('Key "%s" in %s but not in %s' % (str(k), str(dic1), str(dic2)))
