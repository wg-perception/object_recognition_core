#!/usr/bin/env python

import cgi
import cgitb
import tempfile
import sys
import couchdb
import os
from object_recognition import models, dbtools
from object_recognition.ingest.bag_upload import upload_bag
sys.stderr = sys.stdout

form = cgi.FieldStorage()
try:
    bag_item = form['bag_file']
except KeyError:
    print "Content-Type: text/plain\n"
    print "You must supply a bag file."
    sys.exit(-1)

if bag_item.file and bag_item.done:
    obj = models.Object(object_name=form.getfirst('object_name', ''),
                        description=form.getfirst('description', ''),
                        tags=[x.strip() for x in form.getfirst('tags', '').split(',')],
                        author_name=form.getfirst('author_name', 'J. Doe'),
                        author_email=form.getfirst('author_email', 'j.doe@nowhere'),
                        )
    upload_bag(obj,bag_item.file)
    print "Content-Type: text/plain\n"
    print "Upload successful."
else:
    print "Content-Type: text/plain\n"
    print "Could not upload."