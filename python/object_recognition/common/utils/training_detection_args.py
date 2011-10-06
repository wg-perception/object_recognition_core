"""
Module that creates a function to define/read common arguments for the training/detection pipeline
"""

from object_recognition import models, dbtools
from object_recognition.common.utils.parser import ObjectRecognitionParser
import couchdb
import os
import yaml

def read_arguments(parser=None):
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
    if args.object_ids:
        object_ids.extend(args.object_ids[1:-1].split(','))
    if params.get('object_ids', None):
        object_ids.extend(params['object_ids'])

    object_names = []
    if args.object_names:
        object_names.extend(args.object_names[1:-1].split(','))
    if params.get('object_names', None):
        object_names.extend(params['object_names'])
    if object_names:
        for object_name in args.object_names[1:-1].split(','):
            object_ids.extend(models.objects_by_name(db, object_name))

    if args.object_ids == 'all' or args.object_names == 'all' or params['object_ids'] == 'all' or \
                                                params['object_names'] == 'all':
        object_ids = [ obj.id for obj in models.Object.all(db) ]
    params['object_ids'] = list(set(object_ids))

    pipeline_params = []
    for key , value in params.iteritems():
        if key.startswith('pipeline'):
            pipeline_params.append(value)

    return params, args, pipeline_params, args.do_display, db_dict, db
