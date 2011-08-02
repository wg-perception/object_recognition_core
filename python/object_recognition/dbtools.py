DEFAULT_SERVER_URL = 'http://localhost:5984'

def create_db(db_name, couch):
    ''' Create and return a handle to the specified couch db database.
    Will attempt to find the existing db,
    and create if it doesn't already exist.
    @param db_name: The short name for the database, e.g. 'bags','objects','sessions'
    '''
    if db_name in couch:
        db = couch[db_name]
    else:
        db = couch.create(db_name)
    return db

def init_object_databases(couch):
    dbs = dict(bags=create_db('bags',couch),
               objects=create_db('objects',couch),
               sessions=create_db('sessions',couch),
               )
    return dbs