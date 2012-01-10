"""
Module that creates a function to define/read common arguments for the training/detection pipeline
"""

from ecto_object_recognition.object_recognition_db import ObjectDbParameters
from object_recognition import models, dbtools
from object_recognition.common.utils.parser import ObjectRecognitionParser
import os
import sys
import yaml
try:
    import ecto_ros
    ECTO_ROS_FOUND = True
except ImportError:
    ECTO_ROS_FOUND = False

def interpret_object_ids(args, db_params, pipeline_param):
    """
    Given command line arguments and the parameters of the pipeline, clean the 'object_ids' field to be a list of
    object ids
    """
    # initialize the DB
    db = dbtools.db_params_to_db(ObjectDbParameters(db_params))

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
    return list(object_ids)

def common_read_params(extra_pipeline_fields, do_commit):
    parser = ObjectRecognitionParser()
    parser.add_argument('-c', '--config_file', help='Config file')
    parser.add_argument('--object_ids', help='If set, it overrides the list of object_ids in the config file')
    parser.add_argument('--object_names', help='If set, it overrides the list of object names in the config file')
    parser.add_argument('--visualize', help='If set, it will display some windows with temporary results',
                       default=False, action='store_true')

    if do_commit:
        parser.add_argument('--commit', dest='commit', action='store_true',
                        default=False, help='Commit the data to the database.')

    ros_group = parser.add_argument_group('ROS Parameters')
    ros_group.add_argument('--node_name', help='The name for the node. If "", it is not run in a ROS node',
                           default='object_recognition')

    if ECTO_ROS_FOUND:
        original_argv = sys.argv
        clean_args = sys.argv
        ecto_ros.strip_ros_args(clean_args)
        args = parser.parse_args(args=clean_args[1:])

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
    voter_params = {}
    for key , value in params.iteritems():
        if key.startswith('source'):
            source_params[int(key[6:])] = value
        elif key.startswith('pipeline'):
            # check the different fields
            for field in [ 'method', 'submethod', 'package', 'parameters'] + extra_pipeline_fields:
                if field not in value:
                    raise RuntimeError('The pipeline parameters need to have the subfield "%s"' % field)
            pipeline_params[int(key[8:])] = value
        elif key.startswith('sink'):
            sink_params[int(key[4:])] = value
        elif key.startswith('voter'):
            voter_params[int(key[5:])] = value

    return source_params, pipeline_params, sink_params, voter_params, args

def read_arguments_training():
    source_params, pipeline_params, sink_params, _voter_params, args = common_read_params([], True)
    # for each pipeline, get the right object ids
    for _pipeline_id, pipeline_param in pipeline_params.iteritems():
        # clean the object_ids
        pipeline_param['parameters']['object_ids'] = interpret_object_ids(args, pipeline_param['parameters']['db'],
                                                                          pipeline_param['parameters'])

    args = vars(args)
    return source_params, pipeline_params, sink_params, args

def read_arguments_detector():
    source_params, pipeline_params, sink_params, voter_params, args = common_read_params([ 'sources' ], False)

    # for each pipeline, make sure the corresponding source/sink exist
    for _pipeline_id, pipeline_param in pipeline_params.iteritems():
        for cell_type, params in [ ('source', source_params), ('sink', sink_params), ('voter', voter_params) ]:
            for cell_id in pipeline_param.get(cell_type + 's', []):
                if cell_id not in params:
                    raise RuntimeError('The pipeline parameters has an invalid %s number' % type)
        # clean the object_ids
        pipeline_param['parameters']['object_ids'] = interpret_object_ids(args, pipeline_param['parameters']['db'],
                                                                          pipeline_param['parameters'])

    args = vars(args)
    return source_params, pipeline_params, sink_params, voter_params, args
