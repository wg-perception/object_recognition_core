from object_recognition.dbtools import DEFAULT_SERVER_URL, init_object_databases
import couchdb
from couchdb.mapping import TextField, ListField, DateTimeField, Document
from datetime import datetime

class Object(Document):
    object_id = TextField()
    object_name = TextField()
    description = TextField()
    tags = ListField(TextField())
    author_name = TextField()
    author_email = TextField()
    added = DateTimeField(default=datetime.now)

class Session(Document):
    object_id = TextField()
    session_id = TextField()
    description = TextField()
    tags = ListField(TextField())
    author_name = TextField()
    author_email = TextField()
    added = DateTimeField(default=datetime.now)
                            
if __name__ == "__main__":
    couch = couchdb.Server(DEFAULT_SERVER_URL)
    dbs = init_object_databases(couch)
    obj = Object(object_id="TestObject",object_name="A test object.",description="test objects are fun.",tags=['test','object','tod'])
    obj.store(dbs['objects'])
    print obj
    print dbs
