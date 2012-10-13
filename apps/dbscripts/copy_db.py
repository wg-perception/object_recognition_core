#!/usr/bin/env python
import object_recognition
from object_recognition import tools as dbtools
import couchdb
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Copies docs from one db to another.')
    parser.add_argument('--remote', dest='remote', type=str,
                   help='Remote db to copy from.')
    parser.add_argument('--remote_collection',
                   dest='remote_collection', type=str,
                   help='Remote collection.')
    object_recognition.dbtools.add_db_arguments(parser)
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    remote_couch = couchdb.Server(args.remote)
    local_db = dbtools.init_object_databases(couch)
    remote_db = remote_couch[args.remote_collection]
    results = remote_db.view('_all_docs')
    total_docs = len(results)
    i = 1
    for x in results:
      doc = remote_db.get(x.id)
      doc = doc.copy()
      
      attachments = doc.get('_attachments',False)
      if attachments:
        del doc['_attachments']
      if x.id not in local_db:
        (doc_id,rev) = local_db.save(doc)
        if attachments:
          for key,val in attachments.iteritems():
            flo = remote_db.get_attachment(x.id,key)
            doc = local_db.get(x.id)
            local_db.put_attachment(doc,flo,key,val['content_type'])
      print '%d out of %d'%(i,total_docs)
      i += 1

