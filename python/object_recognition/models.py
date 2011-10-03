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
    Type = TextField(default="Object")
    
    all = ViewField('objects', '''\
        function(doc) {
            if(doc.Type == "Object")
                emit(doc.object_name,doc)
        }
    ''')
    by_object_name = ViewField('objects', '''\
        function(doc) {
            if(doc.Type == "Object")
                emit(doc.object_name, doc)
        }
    ''')

    by_tag = ViewField('objects', '''\
        function(doc) {
            if(doc.Type == "Object")
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
    Type = TextField(default="Observation")
    by_object_id = ViewField('observations', '''\
        function(doc) {
        if(doc.Type == "Observation")
            emit(doc.object_id, doc)
        }
    ''')

    by_frame_number = ViewField('observations', '''\
        function(doc) {
            if(doc.Type == "Observation")
                emit(doc.frame_number, doc)
        }
    ''')

    by_session_id = ViewField('observations', '''\
        function(doc) {
            if(doc.Type == "Observation")
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
    Type = TextField(default="Session")
    all = ViewField('sessions', '''\
        function(doc) {
            if(doc.Type == "Session")
                emit(null,doc)
        }
    ''')
    by_object_id = ViewField('sessions', '''\
        function(doc) {
            if(doc.Type == "Session")
                emit(doc.object_id,doc)
        }
    ''')
    by_bag_id = ViewField('sessions', '''\
        function(doc) {
            if(doc.Type == "Session")
                emit(doc.bag_id,doc)
        }
    ''')

    @classmethod
    def sync(cls, db):
        cls.all.sync(db)
        cls.by_object_id.sync(db)
        cls.by_bag_id.sync(db)

class Model(Document):
    object_id = TextField()
    model_params = TextField()
    Type = TextField(default="Model")
    ModelType = TextField()
    
    by_object_id = ViewField('models', '''\
        function(doc) {
            if (doc.Type == "Model")
                emit(doc.object_id, doc)
        }
    ''')
    by_object_id_and_TOD = ViewField('models', '''\
        function(doc) {
            if ((doc.Type == "Model") && (doc.model_type == "TOD"))
                emit(doc.object_id, doc)
        }
    ''')
    all = ViewField('models', '''\
        function(doc) {
            if(doc.Type == "Model")
                emit(null,doc)
        }''')
    @classmethod
    def sync(cls, db):
        cls.by_object_id.sync(db)
        cls.by_object_id_and_TOD.sync(db)
        cls.all.sync(db)

def sync_models(db):
    Object.sync(db)
    Session.sync(db)
    Observation.sync(db)
    Model.sync(db)

def find_all_observations_for_session(observation_collection, session_id):
    ''' Finds all of the observations associated with a session, and returns a list
    of their ids. These are sorted by the frame number,
    so they should be in chronological ordering.
    '''
    #run the view, keyed on the session id.
    results = Observation.by_session_id(observation_collection, key=session_id)
    if len(results) == 0 : return []
    #create a list of tuples, so that they can be sorted by frame number
    obs_tuples = [ (obs.frame_number, obs.id) for obs in results]
    # sort by frame_number, helps preserve chronological order
    obs_ids = zip(*sorted(obs_tuples, key=lambda obs: obs[0]))[1]
    return obs_ids

def find_all_observations_for_object(observation_collection, object_id):
    ''' Finds all of the observations associated with an object, and returns a list
    of their ids. These are sorted by the frame number,
    so they should be in chronological ordering.
    '''
    #run the view, keyed on the session id.
    results = Observation.by_object_id(observation_collection, key=object_id)
    if len(results) == 0 : return []
    #create a list of tuples, so that they can be sorted by frame number
    obs_tuples = [ (obs.frame_number, obs.id) for obs in results]
    # sort by frame_number, helps preserve chronological order
    obs_ids = zip(*sorted(obs_tuples, key=lambda obs: obs[0]))[1]
    return obs_ids

def find_model_for_object(models_collection, object_id, model_type='all'):
    ''' Finds all of the models associated with the given object_id
    The type of the model can be specified through model_type
    '''
    #run the view, keyed on the object id.
    if model_type == 'all':
        r = Model.by_object_id(models_collection, key=object_id)
    elif model_type == 'TOD':
        r = Model.by_object_id_and_TOD(models_collection, key=object_id)
    if len(r) == 0 : return []
    return [ m.id for m in r ]

def objects_by_name(objects_collection, object_name):
    r = Object.by_object_name(objects_collection, key=object_name)
    if len(r) == 0 : return []
    return r

if __name__ == "__main__":
    couch = couchdb.Server(DEFAULT_SERVER_URL)
    dbs = init_object_databases(couch)
    obj = Object(object_id="TestObject", object_name="A test object.", description="test objects are fun.", tags=['test', 'object', 'tod'])
    obj.store(dbs['objects'])
    print obj
    print dbs
