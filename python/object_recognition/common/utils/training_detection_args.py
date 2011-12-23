"""
Module that creates a function to define/read common arguments for the training/detection pipeline
"""

from object_recognition import models, dbtools
from object_recognition.common.utils.parser import ObjectRecognitionParser
from object_recognition.dbtools import args_to_db_params
import os
import sys
import yaml
try:
    import ecto_ros
    ECTO_ROS_FOUND = True
except ImportError:
    ECTO_ROS_FOUND = False

def interpret_object_ids(args, source_param, pipeline_param):
    """
    Given command line arguments and the parameters of the pipeline, clean the 'object_ids' field to be a list of
    object ids
    """
    # read some parameters
    db_params = args_to_db_params(args, source_param)

    # initialize the DB
    db = dbtools.db_params_to_db(db_params)

    # read the object_ids
    object_ids = set()
    for obj in (args.__dict__, pipeline_param):
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
            object_ids.update(ids)
        if names and names != 'missing':
            for object_name in names:
                object_ids.update([str(x.id) for x in models.objects_by_name(db, object_name)])
        # if we got some ids through the command line, just stop here
        if object_ids:
            break
    return object_ids

def read_arguments(parser=None, do_commit=False):
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
    parser.add_argument('--visualize', help='If set, it will display some windows with temporary results',
                       default=False, action='store_true')

    dbtools.add_db_arguments(parser, do_default=False, do_commit=do_commit)

    ros_group = parser.add_argument_group('ROS Parameters')
    ros_group.add_argument('--node_name', help='The name for the node', default='object_recognition')

    if ECTO_ROS_FOUND:
        args = parser.parse_args()

        original_argv = sys.argv
        ecto_ros.strip_ros_args(sys.argv)
        args = parser.parse_args(args=sys.argv[1:])
        
        if args.node_name:
            ecto_ros.init(original_argv, args.node_name, False)
    else:
        args = parser.parse_args()

    # define the input
    if args.config_file is None or not os.path.exists(args.config_file):
        print >> sys.stderr, "The option file does not exist. --help for usage."
        sys.exit(-1)

    params = yaml.load(open(args.config_file))

    # read the different parameters from the config file, for each pipeline
    source_params = {}
    pipeline_params = {}
    sink_params = {}
    for key , value in params.iteritems():
        if key.startswith('source'):
            source_params[int(key[6:])] = value
        elif key.startswith('pipeline'):
            for field in [ 'object_ids', 'source', 'sink', 'method', 'submethod', 'package', 'parameters' ]:
                if field not in value:
                    raise RuntimeError('The pipeline parameters need to have the field "%s"' % field)
            pipeline_params[int(key[8:])] = value
        elif key.startswith('sink'):
            pipeline_params[int(key[4:])] = value

    # for each pipeline, make sure the corresponding source/sink exist
    for _pipeline_id, pipeline_param in pipeline_params.iteritems():
        if pipeline_param['source'] not in source_params:
            raise RuntimeError('The pipeline parameters has an invalid source number')
        if pipeline_param['sink'] not in sink_params:
            raise RuntimeError('The pipeline parameters has an invalid sink number')
        # clean the object_ids
        pipeline_param['object_ids'] = interpret_object_ids(args, source_params[pipeline_param['source']],
                                                            pipeline_param)

    args = vars(args)
    return source_params, pipeline_params, sink_params, args
