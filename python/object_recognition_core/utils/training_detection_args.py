"""
Module that creates a function to define/read common arguments for the training/detection pipeline
"""
from object_recognition_core.utils.parser import ObjectRecognitionParser
import json
import os
import yaml

class OrkConfigurationError(Exception):
    """
    Exception proper to parsing a configuration file
    """
    pass

def create_parser(do_training=False):
    """
    Convenience function for creating a Python argument parser for ORK
    
    :param do_training: if True, it is a parser for a training pipeline: it only adds a commit argument that allows
            you to commit to a database
    """
    parser = ObjectRecognitionParser()
    parser.add_argument('-c', '--config_file', help='Config file')
    parser.add_argument('--visualize', help='If set and the pipeline supports it, it will display some windows '
                        'with temporary results', default=False, action='store_true')

    if do_training:
        parser.add_argument('--commit', dest='commit', action='store_true',
                        default=False, help='Commit the data to the database.')

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

    # make sure we have a dictionary
    if not isinstance(params, dict):
        raise OrkConfigurationError('Your config file must be a JSON string of (key,val) where key: "a_cell_name" and '
                                    'val is the dictionary or its parameters')

    # Go over the different cells
    allowed_keys = set(['type', 'module', 'parameters', 'inputs', 'outputs'])
    for cell_name, cell_params in params.items():
        # Make sure we can find the cell:
        if 'type' not in cell_params or 'module' not in cell_params:
            raise OrkConfigurationError('To find your cell "%s", you must define the parameters ' % cell_name +
                                        '"type" and "module": the ecto cell whose class is "type" in the module '
                                        '"module" can then be loaded.')
        for key_level1, val_level1 in cell_params.items():
            # special case of parameters that is yet another level
            if key_level1 == 'parameters':
                for key_level2, val_level2 in val_level1.items():
                    if isinstance(val_level2, (list, dict)):
                        val_level1[key_level2] = json.dumps(val_level2)
                    # for standard parameters, force a renaming to emphasize the change
                    if key_level2 in ['db', 'object_ids']:
                        val_level1['json_' + key_level2] = json.dumps(val_level2)
                        val_level1.pop(key_level2)
            elif key_level1 in ['inputs', 'outputs']:
                if not isinstance(val_level1, list):
                    raise OrkConfigurationError('The inputs/outputs need to be a list, got %s instead' % 
                                                    val_level2)
                continue
            elif key_level1 not in allowed_keys:
                raise OrkConfigurationError('First level key "%s" must be in ' %key_level1 + ', '.join(allowed_keys))

    return params

def read_arguments(args):
    """
    Given a command line parser, get the parameters from the configuration file

    :param parser: the parsed arguments (after ROS cleanup if needed)
    :return: a tuple (ork_parameters, raw arguments after ROS cleanup). The ork_parameters describe the graph that
            will be run. It is the dict version of the YAML inside the configuration file
    """
    if args.config_file is None or not os.path.exists(args.config_file):
        raise OrkConfigurationError('The option file does not exist. --help for usage.')

    params = read_arguments_from_string(open(args.config_file))

    args = vars(args)

    return params, args
