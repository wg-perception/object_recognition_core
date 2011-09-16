#!/usr/bin/env python
import object_recognition
from object_recognition import dbtools, models
import couchdb
import argparse

from delete_observations import delete_observations

def parse_args():
    parser = argparse.ArgumentParser(description='Delete objects from the database.')
    parser.add_argument('objects', metavar='OBJECT', type=str, nargs='+',
                   help='Object ids to delete.')
    object_recognition.dbtools.add_db_options(parser)
    args = parser.parse_args()
    return args

def delete_object(dbs, object_ids, commit):
    sessions = dbs['sessions']
    observations = dbs['observations']
    objects = dbs['objects']
    bags = dbs['bags']
    for object_id in object_ids:
        ids = [session.id for session in models.Session.by_object_id(sessions, key=object_id)]
        delete_observations(dbs, ids, commit)
        for bag in models.Bag.by_object_id(dbs['bags'], key=object_id):
            print "deleting bag: ", bag.id
            if commit:
                bags.delete(bag)
        print "deleting object: ", object_id
        if commit:
            object = models.Object.load(objects, object_id)
            if object:
                objects.delete(object)


if __name__ == "__main__":
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    dbs = dbtools.init_object_databases(couch)
    delete_object(dbs, args.objects, args.commit)
    if not args.commit:
        print 'just kidding. --commit to actually do it.'
