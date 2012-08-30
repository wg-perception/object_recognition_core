#!/usr/bin/env python
"""
This script tests whether a detection pipeline can be created given a config file.
It is not meant to be run as a test of object_recognition but as a test for and by each 
pipeline independently.
"""

from ecto.opts import scheduler_options
from object_recognition_core.pipelines.detection import DetectionPipeline, DetectionBlackbox, \
    validate_detection_pipeline
from object_recognition_core.pipelines.plasm import create_detection_plasm
from object_recognition_core.utils.find_classes import find_classes
from object_recognition_core.utils.training_detection_args import common_create_parser, read_arguments_detector
import sys

if __name__ == '__main__':
    # create an ORK parser
    parser = common_create_parser()

    # add ecto options
    scheduler_options(parser)

    # read the config file
    source_params, pipeline_params, sink_params, voter_params, args = read_arguments_detector(parser)

    pipelines = find_classes([ pipeline_param['package'] for pipeline_param in pipeline_params.itervalues()],
                               DetectionPipeline) #map of string name to pipeline class

    for _pipeline_id, pipeline_param in pipeline_params.iteritems():
        # make sure object_ids is empty (so that we don't have to deal with the DB
        if 'object_ids' in pipeline_param['parameters']:
            pipeline_param['parameters']['object_ids'] = []
        pipeline = pipelines.get(pipeline_param['method'], False)
        if not pipeline:
            sys.stderr.write('Invalid pipeline name: %s\nMake sure that the pipeline type is defined by a DetectionPipeline class, in the name class function.' % pipeline_param['method'])
            sys.exit(-1)
        # get a pipeline and validate its inputs/outputs
        detector = DetectionBlackbox(pipeline,**pipeline_param)
        if 'sinks' in pipeline_param or 'voters' in pipeline_param:
            validate_detection_pipeline(detector)
    # create the big plasm
    plasm = create_detection_plasm(source_params, pipeline_params, sink_params, voter_params)
