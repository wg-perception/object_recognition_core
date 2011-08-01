#!/usr/bin/env python

import cgi
import cgitb
import tempfile
import sys
import couchdb

sys.stderr = sys.stdout
print "Content-Type: text/plain"
print

'''
    <FORM action="cgi-bin/bag_upload.py" enctype="multipart/form-data" method="post">
      object name: <INPUT type="text" name="object_name"><BR>
      object description:<BR> <TEXTAREA rows=5 cols=20 name="description">A semi detailed description of your object.</TEXTAREA><BR>
      object tags: <INPUT type="text" name="tags"><BR>
      email: <INPUT type="text" name="author_email"><BR>
      Bag file: <INPUT type="file" name="bag_file"><BR>
      <INPUT type="submit" value="Send"> <INPUT type="reset">
    </FORM>
'''

def create_db(db_name, couch):
    if db_name in couch:
        db = couch[db_name]
    else:
        db = couch.create(db_name)
    return db


server_url = 'http://localhost:5984'
couch = couchdb.Server(server_url)

form = cgi.FieldStorage()
try:
  bag_item = form['bag_file']
except KeyError:
  print "You must supply a bag file."
  sys.exit(-1)

if bag_item.file and bag_item.done:
    objects = create_db('objects', couch)
    bags = create_db('bags', couch)
    object = {'object_name':form.getfirst('object_name',''),
              'description':form.getfirst('description',''),
              'tags':[x.strip() for x in form.getfirst('tags','').split(',')],
              'author_email': form.getfirst('author_email','noone@nowhere'),
              }
    doc_id,doc_rev = objects.save(object)
    bag = {'object_id':doc_id}
    bags.save(bag)
    bags.put_attachment(doc=bag, content=bag_item.file, filename='data.bag', content_type='application/octet-stream')
    print "Thank you for your upload."
else:
    print "Your upload was not finished."
