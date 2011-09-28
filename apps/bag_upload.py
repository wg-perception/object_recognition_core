#!/usr/bin/env python
import roscompat
from object_recognition import models, dbtools
from object_recognition.ingest.bag_upload import upload_bag
import argparse
from object_recognition.dbtools import add_db_options

def parse_args():
    parser = argparse.ArgumentParser(description='''Uploads a bag, with an object description to the db.''',
                                      fromfile_prefix_chars='@')
    parser.add_argument('-i', '--input', metavar='BAG_FILE', dest='bag', type=str,
                       default='',
                       help='A bagfile to upload.')
    parser.add_argument('-n', '--object_name', metavar='OBJECT_NAME', dest='object_name', type=str,
                       default='')
    parser.add_argument('-d', '--description', metavar='DESCRIPTION', dest='description', type=str,
                       default='')
    parser.add_argument('-a', '--author', metavar='AUTHOR_NAME', dest='author_name', type=str,
                       default='')
    parser.add_argument('-e', '--email', metavar='EMAIL_ADDRESS', dest='author_email', type=str,
                       default='')
    parser.add_argument('tags', metavar='TAGS', type=str, nargs='+',
                   help='Tags to add to object description.')
    add_db_options(parser)
    args = parser.parse_args()
    if len(args.bag) < 1:
      parser.print_help()
      sys.exit(1)
    return args

if "__main__" == __name__:
    args = parse_args()
    bag = open(args.bag, 'rb')
    obj = models.Object(object_name=args.object_name,
                        description=args.description,
                        tags=args.tags,
                        author_name=args.author_name,
                        author_email=args.author_email,
                        )
    if args.commit:
        bag_up = upload_bag(obj, bag, couchdb_url=args.db_root)
        print "Uploaded bag has id =", bag_up.id
    else:
        print 'Did not upload. Please pass --commit to actually upload the bag.'

