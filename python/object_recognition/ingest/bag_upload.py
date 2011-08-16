#!/usr/bin/env python

import tempfile
import sys
import couchdb
import os
from object_recognition import models, dbtools
sys.stderr = sys.stdout

def upload_bag(obj, bag_file, couchdb_url=dbtools.DEFAULT_SERVER_URL):
    ''' Uploads a bag to the couch of your choice, associates it with the object description given
    @param obj: An instance of object_rocognition.models.Object
    @param bag_file: A file like object that looks like a ros bag.
    '''

    couch = couchdb.Server(couchdb_url)
    dbs = dbtools.init_object_databases(couch)
    objects = dbs['objects']
    bags = dbs['bags']
    obj.store(objects)
    bag = models.Bag(object_id=obj.id,
                     author_name=obj.author_name,
                     author_email=obj.author_email,
                     )
    bag.store(bags)
    bags.put_attachment(doc=bag, content=bag_file, filename='data.bag', content_type='application/octet-stream')
    return bag
