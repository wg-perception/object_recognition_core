#!/usr/bin/env python
"""
This script tests whether a detection pipeline can be created given a config file.
It is not meant to be run as a test of object_recognition but as a test for and by each 
pipeline independently.
"""

from object_recognition_core.utils.training_detection_args import common_create_parser, common_parse_config_file
from object_recognition_core.utils.find_classes import find_classes
from object_recognition_core.pipelines.detection import DetectionPipeline, validate_detection_pipeline

if __name__ == '__main__':
    # read the config file
    parser = common_create_parser()
    args = parser.parse_args()
    source_params, pipeline_params, sink_params, voter_params = common_parse_config_file(args.config_file, [])

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
        detector = pipeline().detector(**pipeline_param)
        if 'sinks' in pipeline_param or 'voters' in pipeline_param:
            validate_detection_pipeline(detector)
