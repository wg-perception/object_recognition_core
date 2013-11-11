#!/usr/bin/env python
# Script that adds an object to the database

from __future__ import print_function
import object_recognition_core.db.tools as dbtools
import couchdb
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Delete objects from the database.')
    parser.add_argument('objects', metavar='OBJECTS', type=str, nargs='+',
                   help='Object ids to delete.')
    dbtools.add_db_arguments(parser)
    args = parser.parse_args()
    return args

def deletion_view(db):
    view = couchdb.design.ViewDefinition('object_recognition', 'refer_object_id', '''function(doc) {
         if (doc.object_id)
             emit(doc.object_id, doc);
    }''')
    view.sync(db)

def delete_object(db, object_ids, commit):
    for object_id in object_ids:
        for row in db.view('object_recognition/refer_object_id', key=object_id):
            print('Deleting doc: %s of type "%s" referring to object: %s' % (row.id, row.value['Type'], row.key))
            if commit:
                del db[row.id]
        if not db.get(object_id):
             print("No object by id", object_id, "found!")
        elif commit:
            print("Deleting object:", object_id)
            del db[object_id]

if __name__ == "__main__":
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    db = dbtools.init_object_databases(couch)
    deletion_view(db)
    delete_object(db, args.objects, args.commit)
    if not args.commit:
        print('just kidding. --commit to actually do it.')
