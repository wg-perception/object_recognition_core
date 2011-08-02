#!/usr/bin/env python

import cgi
import cgitb
import tempfile
import sys
import couchdb
import os
from object_recognition import models, dbtools
sys.stderr = sys.stdout

couch = couchdb.Server(dbtools.DEFAULT_SERVER_URL)
dbs = dbtools.init_object_databases(couch)

form = cgi.FieldStorage()
try:
    bag_item = form['bag_file']
except KeyError:
    print "Content-Type: text/plain\n"
    print "You must supply a bag file."
    sys.exit(-1)

if bag_item.file and bag_item.done:
    objects = dbs['objects']
    bags = dbs['bags']
    obj = models.Object(object_name=form.getfirst('object_name', ''),
                        description=form.getfirst('description', ''),
                        tags=[x.strip() for x in form.getfirst('tags', '').split(',')],
                        author_name=form.getfirst('author_name', 'J. Doe'),
                        author_email=form.getfirst('author_email', 'j.doe@nowhere'),
                        )
    obj.store(objects)
    bag = models.Bag(object_id=obj.id,
                     author_name=obj.author_name,
                     author_email=obj.author_email,
                     )
    bag.store(bags)
    bags.put_attachment(doc=bag, content=bag_item.file, filename='data.bag', content_type='application/octet-stream')
    print "Content-Type: text/plain\n"
    print "Upload successful."
else:
    print "Content-Type: text/plain\n"
    print "Could not upload."