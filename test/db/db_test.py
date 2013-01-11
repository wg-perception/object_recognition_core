#!/usr/bin/env python

from object_recognition_core.db import ObjectDbTypes, ObjectDb, ObjectDbParameters
from object_recognition_core.db.tools import db_params_to_db
from object_recognition_core.db.object_db import core_db_types

print 'Existing core types: ' + str(core_db_types())

str_to_enum = {'CouchDB': ObjectDbTypes.COUCHDB, 'filesystem': ObjectDbTypes.FILESYSTEM, 'empty': ObjectDbTypes.EMPTY }

# test default parameters
for db_params_raw in [{'type': 'CouchDB', 'root': 'http://localhost:5984', 'collection': 'object_recognition'},
                      {'path': '/tmp', 'type': 'filesystem', 'collection': 'object_recognition'},
                      {'type': 'empty'}]:
    type_str = db_params_raw['type']
    print 'starting type ' + type_str
    db_params = ObjectDbParameters(db_params_raw)

    db = ObjectDb(db_params)
    db_params_new = db.parameters().raw

    for dic1, dic2 in [(db_params_raw, db_params_new), (db_params_new, db_params_raw)]:
        for k, v in dic1.items():
            if (k not in dic2) or (k in dic2 and dic2[k] != v):
                raise RuntimeError('Key "%s" in %s but not in %s' % (str(k), str(dic1), str(dic2)))

    if str_to_enum[type_str] != db_params.type:
        raise RuntimeError('The "type" argument in db_params are wrong for db of type %s' % type_str)

    print 'ending type ' + type_str

    # test that we can convert a JSON string to an ObjectDbParameters type
    db_params_to_db(db_params)

print 'all good'
