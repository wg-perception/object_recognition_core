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
    dbs = dict(bags=create_db('bags', couch),
               objects=create_db('objects', couch),
               sessions=create_db('sessions', couch),
               observations=create_db('observations', couch),
               models=create_db('models', couch),
               )
    return dbs

def add_db_options(parser):
    '''Appends some common arguments to the argparse parser.
    db_root will contain the server url.
    commit will contain a bool, that indicates whether anything should be changed in the database.
    '''
    parser.add_argument('--db_root', metavar='DB_ROOT_URL', dest='db_root', type=str, default=DEFAULT_SERVER_URL,
                       help='The database root URL to connect to. e.g. %s or http://foo:5984' % DEFAULT_SERVER_URL,
                       required=True)
    parser.add_argument('--commit', dest='commit', action='store_true',
                        default=False, help='Commit the data to the database.')
