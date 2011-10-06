"""
Module that creates a function to define/read common arguments for the training/detection pipeline
"""

from object_recognition import models, dbtools
from object_recognition.common.utils.parser import ObjectRecognitionParser
import couchdb
import os
import yaml

def read_arguments(parser=None, training=False):
    """
    Returns:
    params, pipeline_params, db_dict, db
    params: all the pipeline parameters specified in the config file or command line
    pipeline_params: an array of parameters for each pipeline
    db_dict: the dictionary of the db parameters
    db: a db object created from those parameters
    """
    if parser is None:
        parser = ObjectRecognitionParser()

    parser.add_argument('-c', '--config_file', help='Config file')
    parser.add_argument('--object_ids', help='If set, it overrides the list of object_ids in the config file')
    parser.add_argument('--object_names', help='If set, it overrides the list of object names in the config file')
    parser.add_argument('--do_display', help='If set, it will display some windows with temporary results',
                       default=False, action='store_true')
    if training:
        parser.add_argument('--recompute', help='If specified then all models requested will be recomputed.', default=False, action='store_true')
    args = parser.parse_args()

    # define the input
    if args.config_file is None or not os.path.exists(args.config_file):
        raise 'option file does not exist'

    params = yaml.load(open(args.config_file))

    # read some parameters
    db_dict = params['db']

    # initialize the DB
    if db_dict['type'].lower() == 'couchdb':
        db = dbtools.init_object_databases(couchdb.Server(db_dict['root']))

    # read the object_ids
    object_ids = []
    for obj in (args.__dict__, params):
        ids = obj.get('object_ids', None)
        names = obj.get('object_names', None)
        if 'all' in (ids, names):
            object_ids = [ str(x.id) for x in models.Object.all(db) ] #unicode without the str()
        elif ids:
            object_ids = ids[1:-1].split(',')
            break
        elif names:
            for object_name in names[1:-1].split(','):
                object_ids += [str(x.id) for x in models.objects_by_name(db, object_name)]
            break
    object_ids = list(set(object_ids))
    if not getattr(args, 'recompute', True): #strip out all precomputed models
        for m in models.Model.by_object_id_and_TOD(db):
            if str(m.object_id) in object_ids:
                object_ids.remove(m.object_id)
    print "computing for", object_ids
    params['object_ids'] = object_ids

    pipeline_params = []
    for key , value in params.iteritems():
        if key.startswith('pipeline'):
            pipeline_params.append(value)

    return params, args, pipeline_params, args.do_display, db_dict, db
