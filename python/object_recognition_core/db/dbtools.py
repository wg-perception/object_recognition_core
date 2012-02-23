from object_recognition_core.db.interface import ObjectDbParameters

DEFAULT_DB_COLLECTION = 'object_recognition'
DEFAULT_DB_ROOT = 'http://localhost:5984'
DEFAULT_DB_TYPE = 'CouchDB'

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

def init_object_databases(couch, db_name='object_recognition'):
    db = create_db('object_recognition', couch)
    import models
    models.sync_models(db)
    return db

def add_db_arguments(parser, do_default=True, do_commit=True):
    '''Appends some common arguments to the argparse parser.
    db_type will contain the type of DB (CouchDB, SQL ...).
    db_root will contain the server url.
    '''
    group = parser.add_argument_group('Database Parameters')
    if do_default:
        group.add_argument('--db_type', metavar='DB_TYPE', dest='db_type', type=str, choices=['CouchDB'],
                           default=DEFAULT_DB_TYPE,
                           help='The type of database used: one of [%(choices)s]. Default: %(default)s')
        group.add_argument('--db_root', metavar='DB_ROOT_URL', dest='db_root', type=str, default=DEFAULT_DB_ROOT,
                           help='The database root URL to connect to. Default: %(default)s')
        group.add_argument('--db_collection', metavar='DB_COLLECTION', dest='db_collection', type=str,
                           default=DEFAULT_DB_COLLECTION,
                           help='The database root URL to connect to. Default: %(default)s')
    else:
        group.add_argument('--db_type', metavar='DB_TYPE', dest='db_type', type=str, choices=['CouchDB'],
                       help='The type of database used: one of [%(choices)s].')
        group.add_argument('--db_root', metavar='DB_ROOT_URL', dest='db_root', type=str,
                       help='The database root URL to connect to.')
        group.add_argument('--db_collection', metavar='DB_COLLECTION', dest='db_collection', type=str,
                       default='object_recognition', help='The database root URL to connect to.')
    if do_commit:
        group.add_argument('--commit', dest='commit', action='store_true',
                        default=False, help='Commit the data to the database.')
    return group

def args_to_db_params(args, secondary_parameters={}):
    """
    Given command line parsed args, create an ObjectDbParameters object. The keys in args have to be:
    'db_type', 'db_root', 'db_collection'
    Any parameter that is not in the args will be taken from the dictionary secondary_parameters, where the keys are:
    'type', 'url', 'collection'
    """
    dic = {}
    remap_dic = {'db_type':'type', 'db_root':'root', 'db_collection': 'collection'}
    for args_key, secondary_key in remap_dic.iteritems():
        if hasattr(args, args_key):
            dic[secondary_key] = getattr(args, args_key)
        if secondary_parameters.has_key(secondary_key):
            dic[secondary_key] = secondary_parameters[secondary_key]
    return ObjectDbParameters(dic)

def db_params_to_db(db_params):
    """
    Given a ObjectDbParameters, return  db object
    """
    if db_params.type.lower() == 'couchdb':
        import couchdb
        return init_object_databases(couchdb.Server(db_params.root))
