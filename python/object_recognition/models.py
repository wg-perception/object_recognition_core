from object_recognition.dbtools import DEFAULT_SERVER_URL, init_object_databases
import couchdb
from couchdb.mapping import TextField, ListField, DateTimeField, Document, \
    ViewField, IntegerField
from datetime import datetime

class Object(Document):
    object_name = TextField()
    description = TextField()
    tags = ListField(TextField())
    author_name = TextField()
    author_email = TextField()
    added = DateTimeField(default=datetime.now)

    all = ViewField('objects', '''\
        function(doc) {
            emit(null,doc)
        }
    ''')
    by_object_name = ViewField('objects', '''\
        function(doc) {
            emit(doc.object_name, doc)
        }
    ''')

    by_tag = ViewField('objects', '''\
        function(doc) {
            for( tag in doc.tags )
            {
                emit(doc.tags[tag], doc)
            }
        }
    ''')
    @classmethod
    def sync(cls, db):
        cls.all.sync(db)
        cls.by_object_name.sync(db)
        cls.by_tag.sync(db)

class Observation(Document):
    object_id = TextField()
    session_id = TextField()
    frame_number = IntegerField()

    by_object_id = ViewField('observations', '''\
        function(doc) {
            emit(doc.object_id, doc)
        }
    ''')

    by_frame_number = ViewField('observations', '''\
        function(doc) {
            emit(doc.frame_number, doc)
        }
    ''')

    by_session_id = ViewField('observations', '''\
        function(doc) {
            emit(doc.session_id, doc)
        }
    ''')

    @classmethod
    def sync(cls, db):
        cls.by_session_id.sync(db)
        cls.by_object_id.sync(db)
        cls.by_frame_number.sync(db)

class Session(Document):
    object_id = TextField()
    bag_id = TextField()
    added = DateTimeField(default=datetime.now)

    by_object_id = ViewField('sessions', '''\
        function(doc) {
            emit(doc.object_id,doc)
        }
    ''')
    by_bag_id = ViewField('sessions', '''\
        function(doc) {
            emit(doc.bag_id,doc)
        }
    ''')


    @classmethod
    def sync(cls, db):
        cls.by_object_id.sync(db)
        cls.by_bag_id.sync(db)


class Bag(Document):
    object_id = TextField()
    author_name = TextField()
    author_email = TextField()
    added = DateTimeField(default=datetime.now)
    by_object_id = ViewField('bags', '''\
        function(doc) {
            emit(doc.object_id,doc)
        }
    ''')
    all = ViewField('bags', '''\
        function(doc) {
            emit(null,doc)
        }
    ''')

    @classmethod
    def sync(cls, db):
        cls.all.sync(db)
        cls.by_object_id.sync(db)

def sync_models(dbs):
    Bag.sync(dbs['bags'])
    Object.sync(dbs['objects'])
    Session.sync(dbs['sessions'])
    Observation.sync(dbs['observations'])
if __name__ == "__main__":
    couch = couchdb.Server(DEFAULT_SERVER_URL)
    dbs = init_object_databases(couch)
    obj = Object(object_id="TestObject", object_name="A test object.", description="test objects are fun.", tags=['test', 'object', 'tod'])
    obj.store(dbs['objects'])
    print obj
    print dbs
