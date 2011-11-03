#!/usr/bin/env python
import object_recognition
from object_recognition import dbtools, models
import couchdb
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Delete objects from the database.')
    parser.add_argument('objects', metavar='OBJECT', type=str, nargs='+',
                   help='Object ids to delete.')
    object_recognition.dbtools.add_db_options(parser)
    args = parser.parse_args()
    return args

def deletion_view(db):
    view = couchdb.design.ViewDefinition('object_recognition', 'refer_object_id', '''function(doc) {
         if (doc.object_id)
             emit(doc.object_id, null);
    }''')
    view.sync(db)

def delete_object(db, object_ids, commit):
    for object_id in object_ids:
        for row in db.view('object_recognition/refer_object_id', key=object_id):
            print "Deleting doc:", row.id, 'referring to object:', row.key
            if commit:
                del db[row.id]
        if not db.get(object_id):
             print "No object by id", object_id, "found!"
        elif commit:
            print "Deleting object:", object_id
            del db[object_id]

if __name__ == "__main__":
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    db = dbtools.init_object_databases(couch)
    deletion_view(db)
    delete_object(db, args.objects, args.commit)
    if not args.commit:
        print 'just kidding. --commit to actually do it.'
