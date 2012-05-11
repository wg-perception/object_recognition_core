#!/usr/bin/env python

import couchdb
import sys

def create_db(db_name,couch):
    if db_name in couch:
        db = couch[db_name]
    else:
        db = couch.create(db_name)
    return db

server_url = 'http://localhost:5984'
couch = couchdb.Server(server_url)

bag_name = sys.argv[1]
db = create_db('bags',couch)
doc = { 'bag': bag_name.split('/')[-1] }
db.save(doc)
db.put_attachment(doc=doc,content=open(bag_name),filename='data.bag',content_type='application/octet-stream')
couch.delete('bags')