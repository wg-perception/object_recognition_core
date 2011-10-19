"""
Module that creates a function to define/read common arguments for the training/detection pipeline
"""

from ecto_object_recognition.object_recognition_db import ObjectDbParameters
from object_recognition import models, dbtools
from object_recognition.common.utils.parser import ObjectRecognitionParser
import couchdb
import os
import yaml

def read_arguments(parser=None, argv=None):
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
    args = parser.parse_args(args=argv)

    # define the input
    if args.config_file is None or not os.path.exists(args.config_file):
        raise 'option file does not exist'

    params = yaml.load(open(args.config_file))

    # read some parameters
    db_params = ObjectDbParameters(params['db'])

    # initialize the DB
    if db_params.type.lower() == 'couchdb':
        db = dbtools.init_object_databases(couchdb.Server(db_params.root))

    # read the object_ids
    object_ids = set()
    for obj in (args.__dict__, params):
        ids = obj.get('object_ids', None)
        names = obj.get('object_names', None)
        if 'all' in (ids, names):
            object_ids = set([ str(x.id) for x in models.Object.all(db) ]) #unicode without the str()
            break
        if 'missing' in (ids, names):
            tmp_object_ids = set([ str(x.id) for x in models.Object.all(db) ])
            tmp_object_ids_from_names = set([ str(x.object_id) for x in models.Model.all(db) ])
            object_ids.update(tmp_object_ids.difference(tmp_object_ids_from_names))
        if ids and ids != 'missing':
            object_ids.update(ids[1:-1].split(','))
        if names and names != 'missing':
            for object_name in names[1:-1].split(','):
                object_ids.update([str(x.id) for x in models.objects_by_name(db, object_name)])
        # if we got some ids through the command line, just stop here
        if object_ids:
            break

    object_ids = list(object_ids)
    print "computing for", object_ids
    params['object_ids'] = object_ids

    pipeline_params = []
    for key , value in params.iteritems():
        if key.startswith('pipeline'):
            pipeline_params.append(value)

    return params, args, pipeline_params, args.do_display, db_params, db
