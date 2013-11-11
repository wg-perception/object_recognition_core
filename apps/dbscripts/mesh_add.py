#!/usr/bin/env python
# Script that adds a mesh to an object in the database

from __future__ import print_function 
import argparse
from object_recognition_core.db import models
from object_recognition_core.db.tools import args_to_db_params
import object_recognition_core.db.tools as dbtools
import couchdb

def parse_args():
    parser = argparse.ArgumentParser(description='Add a mesh to an object.', fromfile_prefix_chars='@')
    parser.add_argument('object_id', metavar='OBJECT_ID', type=str, help='ID of the object to add a mesh to.')
    parser.add_argument('mesh_original', metavar='MESH_ORIGINAL', type=str, help='Original mesh of the object.')
    dbtools.add_db_arguments(parser)

    args = parser.parse_args()

    return args

if __name__ == '__main__':
    args = parse_args()

    couch = couchdb.Server(args.db_root)
    db = dbtools.init_object_databases(couch)
    if args.commit:
        dbtools.upload_mesh(db, args.object_id, args.mesh_original, cloud_path=None, mesh_path=None)
        print('Stored mesh for object id :', args.object_id)
    else:
        print('Use the --commit option to commit the proposed change.')
