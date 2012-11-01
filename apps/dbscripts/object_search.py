#!/usr/bin/env python
import object_recognition_core
from object_recognition_core.db import tools as dbtools, models
import couchdb
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Search for an objects in the DB based on tags..')
    parser.add_argument('-t', '--tag', metavar='TAG', dest='tag', type=str, default='',
                       help='Tag to search for.')
    dbtools.add_db_arguments(parser)
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    db = dbtools.init_object_databases(couch)
    if len(args.tag) > 0:
        results = models.Object.by_tag(db, key=args.tag)
    else:
        results = models.Object.by_object_name(db)
    for obj in results:
        print "******************************"
        print "Object Name:", obj.object_name
        print "Description:", obj.description
        print "Tags:", ','.join(obj.tags)
        print "Author:", obj.author_name
        print "email:", obj.author_email
        print "db id:", obj.id
        print "session ids:", [session.id for session in models.Session.by_object_id(db, key=obj.id)]
