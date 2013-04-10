from couchdb.design import ViewDefinition
from couchdb.mapping import TextField, ListField, DateTimeField, Document, ViewField, IntegerField
from datetime import datetime
import object_recognition_core.db.tools as dbtools
from object_recognition_core.db.tools import DEFAULT_DB_ROOT, init_object_databases
import couchdb

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
    method = TextField()

    # this dictionary will include several views: the key will be the type of objectmodel
    by_object_id_and = {}
    by_object_id = ViewField('models', '''\
        function(doc) {
            if (doc.Type == "Model")
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
        cls.all.sync(db)
        # figure out the different models in the DB
        models = [ m.value for m in db.query('''
        function(doc) {
            if (doc.Type == "Model") {
                if (doc.method) {
                    emit(doc.method, doc.method);
                } else {
                    if (doc.ModelType) {
                        emit(doc.ModelType, doc.ModelType);
                    }
                }
            }
        }
        ''', '''
        function(keys, values, rereduce) {
        log(keys);
            var o = {}, key, key_id;
            if (rereduce) {
                for (v in values) {
                    for(j in values[v]) {
                        o[values[v][j]] = values[v][j];
                    }
                }
            } else {
                for (key_id in keys) {
                    key = keys[key_id][0];
                    o[key] = key;
                }
            }
            var r = [];
            for (key in o) {
                r.push(o[key]);
            }
            return r;
        }
        ''') ]

        if not models:
            return
        else:
            models = models[0]
        # for each, create a view and sync it with the DB
        for model in models:
            cls.by_object_id_and[model] = ViewDefinition('models', 'by_object_id_and_' + model, '''\
                function(doc) {
                    if (doc.Type == "Model")
                        if ((doc.method == "%s") || (doc.ModelType == "%s"))
                            emit(doc.object_id, doc)
                }
            ''' % (model, model), wrapper=cls._wrap_row)
            cls.by_object_id_and[model].sync(db)

def sync_models(db):
    Object.sync(db)
    Session.sync(db)
    Observation.sync(db)
    Model.sync(db)

def find_all_observations_for_session(db_params, session_id):
    ''' Finds all of the observations associated with a session, and returns a list
    of their ids. These are sorted by the frame number,
    so they should be in chronological ordering.
    '''
    db = dbtools.db_params_to_db(db_params)
    #run the view, keyed on the session id.
    results = Observation.by_session_id(db, key=session_id)
    if len(results) == 0 : return []
    #create a list of tuples, so that they can be sorted by frame number
    obs_tuples = [ (obs.frame_number, obs.id) for obs in results]
    # sort by frame_number, helps preserve chronological order
    obs_ids = zip(*sorted(obs_tuples, key=lambda obs: obs[0]))[1]
    return obs_ids

def find_all_sessions_for_object(db_params, object_id):
    db = dbtools.db_params_to_db(db_params)
    sessions = Session.by_object_id(db, key=object_id)
    sessions_by_date_added = []
    for x in sessions:
        sess = db[x.id]
        sessions_by_date_added.append((sess['added'], x.id))
    sessions_by_date_added = sorted(sessions_by_date_added)
    tmp = zip(*sessions_by_date_added)
    if tmp:
        return tmp[1]
    else:
        return []

def find_all_observations_for_object(db_params, object_id):
    ''' Finds all of the observations associated with an object, and returns a list
    of their ids. These are sorted by the frame number,
    so they should be in chronological ordering.
    '''
    db = dbtools.db_params_to_db(db_params)
    sessions = find_all_sessions_for_object(db, object_id)
    #run the view, keyed on the session id.
    results = Observation.by_session_id(db, key=sessions[-1])
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
    if not Model.by_object_id_and.has_key(model_type):
        return []
    r = Model.by_object_id_and[model_type](models_collection, key=object_id)

    if len(r) == 0 : return []
    return [ m.id for m in r ]

def objects_by_name(objects_collection, object_name):
    r = Object.by_object_name(objects_collection, key=object_name)
    if len(r) == 0 : return []
    return r

if __name__ == "__main__":
    couch = couchdb.Server(DEFAULT_DB_ROOT)
    dbs = init_object_databases(couch)
    obj = Object(object_id="TestObject", object_name="A test object.", description="test objects are fun.", tags=['test', 'object', 'tod'])
    obj.store(dbs['objects'])
    print obj
    print dbs
