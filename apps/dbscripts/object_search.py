#!/usr/bin/env python
import object_recognition
from object_recognition import dbtools, models
import couchdb
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Search for an objects in the DB based on tags..')
    parser.add_argument('-t', '--tag', metavar='TAG', dest='tag', type=str, default='',
                       help='Tag to search for.')
    object_recognition.dbtools.add_db_options(parser)
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    dbs = dbtools.init_object_databases(couch)
    objects = dbs['objects']
    if len(args.tag) > 0:
        results = models.Object.by_tag(objects, key=args.tag)
    else:
        results = models.Object.by_object_name(objects)
    for obj in results:
        print "******************************"
        print "Object Name:", obj.object_name
        print "Description:", obj.description
        print "Tags:", ','.join(obj.tags)
        print "Author:", obj.author_name
        print "email:", obj.author_email
        print "db id:", obj.id
        print "bag ids:", [bag.id for bag in models.Bag.by_object_id(dbs['bags'], key=obj.id)]
        print "session ids:", [session.id for session in models.Session.by_object_id(dbs['sessions'], key=obj.id)]
