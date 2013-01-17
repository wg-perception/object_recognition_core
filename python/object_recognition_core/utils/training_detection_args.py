"""
Module that creates a function to define/read common arguments for the training/detection pipeline
"""

from object_recognition_core.boost.interface import ObjectDbParameters
from object_recognition_core.db import models, tools as dbtools
from object_recognition_core.db.object_db import core_db_types
from object_recognition_core.utils.parser import ObjectRecognitionParser
import json
import os
import sys
import yaml

# set a global variable defining whether ROS is used
try:
    import ecto_ros
    ECTO_ROS_FOUND = True
except ImportError:
    ECTO_ROS_FOUND = False

class OrkConfigurationError(Exception):
    """
    Exception proper to parsing a configuration file
    """
    pass

def common_interpret_object_ids(pipeline_param_full, args=None):
    """
    Given command line arguments and the parameters of the pipeline, clean the 'object_ids' field to be a list of
    object ids
    """
    pipeline_param = pipeline_param_full['parameters']

    # read the object_ids
    object_ids = None
    if args:
        objs = [args.__dict__, pipeline_param]
    else:
        objs = [pipeline_param]

    for obj in objs:
        ids = obj.get('object_ids', None)
        names = obj.get('object_names', None)

        if ids is None and names is None:
            continue
        
        db_params = pipeline_param.get('db', {})
        db_type = db_params.get('type', '')
        if db_type.lower() not in core_db_types():
            continue

        # initialize the DB
        if isinstance(ids, str) and ids != 'all' and ids != 'missing':
            ids = eval(ids)
        if isinstance(names, str) and names != 'all' and names != 'missing':
            names = eval(names)

        if not ids and not names:
            break

        if object_ids is None:
            object_ids = set()

        db = dbtools.db_params_to_db(ObjectDbParameters(db_params))
        if 'all' in (ids, names):
            object_ids = set([ str(x.id) for x in models.Object.all(db) ])  # unicode without the str()
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
    if isinstance(object_ids, set):
        pipeline_param['object_ids'] = list(object_ids)
    else:
        pipeline_param['object_ids'] = []

def create_parser(do_training=False):
    """
    Convenience function for creating a Python argument parser for ORK
    
    :param do_training: if True, it is a parser for a training pipeline: it only adds a commit argument that allows
            you to commit to a database
    """
    parser = ObjectRecognitionParser()
    parser.add_argument('-c', '--config_file', help='Config file')
    parser.add_argument('--object_ids', help='If set, it overrides the list of object_ids in the config file. E.g.: \'["1ef112f21f12"]\'')
    parser.add_argument('--object_names', help='If set, it overrides the list of object names in the config file')

    if do_training:
        parser.add_argument('--commit', dest='commit', action='store_true',
                        default=False, help='Commit the data to the database.')

    if ECTO_ROS_FOUND:
        # add ROS parameters if ROS was found
        def filter_node_name(node_name):
            return node_name
        ros_group = parser.add_argument_group('ROS parameters')
        ros_group.add_argument('--node_name', help='The name for the node. If "", it is not run in a ROS node',
                           default='object_recognition', type=filter_node_name)

    return parser

def read_arguments_from_string(parameter_str):
    """
    Reads the description of an ORK graph to run from a string that contains the JSON description of it
    
    :param parameter_str: the JSON description of the graph to run as a string
    :return: the parsed parameters as a key value dictionary. VERY IMPORTANT: If a value is an array or another
    dictionary, it is converted to a JSON string (so that we only have one level of hierarchy). The main reason is
    so that those parameters are easily exposed to the user/GUI.
    """
    try:
        params = yaml.load(parameter_str)
    except yaml.parser.ParserError as err:
        raise OrkConfigurationError('The configuration string is not yaml: %s' % err)

    if not params:
        raise OrkConfigurationError('The configuration parameters cannot be empty')

    for _key_level0, val_level0 in params.items():
        for key_level1, val_level1 in val_level0.items():
            # special case of parameters that is yet another level
            if key_level1 == 'parameters':
                for key_level2, val_level2 in val_level1.items():
                    if isinstance(val_level2, (list, dict)):
                        val_level1[key_level2] = json.dumps(val_level2)
                    # for standard parameters, force a renaming to emphasize the change
                    if key_level2 in ['db', 'object_ids', 'submethod']:
                        val_level1['json_' + key_level2] = json.dumps(val_level2)
                        val_level1.pop(key_level2)
            elif key_level1 in ['inputs', 'outputs']:
                if not isinstance(val_level1, list):
                    raise OrkConfigurationError('The inputs/outputs need to be a list, got %s instead' % 
                                                    val_level2)
                continue
            elif isinstance(val_level1, (list, dict)):
                val_level0[key_level1] = json.dumps(val_level1)

    return params

def read_arguments(parser):
    """
    Given a command line parser, get the parameters from the configuration file

    :param parser: an argparse parser
    :return: a tuple (ork_parameters, raw arguments after ROS cleanup). The ork_parameters describe the graph that
            will be run. It is the dict version of the YAML inside the configuration file
    """
    if ECTO_ROS_FOUND:
        original_argv = sys.argv
        clean_args = sys.argv
        ecto_ros.strip_ros_args(clean_args)
        args = parser.parse_args(args=clean_args[1:])

        if args.node_name and args.node_name != '""':
            ecto_ros.init(original_argv, args.node_name, False)
    else:
        args = parser.parse_args()

    if args.config_file is None or not os.path.exists(args.config_file):
        raise OrkConfigurationError('The option file does not exist. --help for usage.')

    params = read_arguments_from_string(open(args.config_file))

    args = vars(args)

    return params, args
