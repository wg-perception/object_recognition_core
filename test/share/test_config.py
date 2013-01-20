#!/usr/bin/env python
"""
This script tests whether a detection pipeline can be created given a config file.
It is not meant to be run as a test of object_recognition but as a test for and by each 
pipeline independently.
"""

from ecto.opts import scheduler_options
from object_recognition_core.pipelines.plasm import create_plasm
from object_recognition_core.utils.training_detection_args import create_parser, read_arguments

if __name__ == '__main__':
    # create an ORK parser
    parser = create_parser()

    # add ecto options
    scheduler_options(parser)

    # read the config file
    args = parser.parse_args()
    ork_params, args = read_arguments(args)

    # override the database parameters
    for cell_name, parameters in ork_params.items():
        if 'parameters' in parameters and 'object_ids' in parameters['parameters']:
            parameters['parameters']['object_ids'] = '[]'

    # create the big plasm
    plasm = create_plasm(ork_params)
