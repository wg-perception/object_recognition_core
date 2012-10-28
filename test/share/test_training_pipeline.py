#!/usr/bin/env python
"""
This script tests whether a detection pipeline can be created given a config file.
It is not meant to be run as a test of object_recognition but as a test for and by each 
pipeline independently.
"""

import sys
from object_recognition_core.db.object_db import ObjectDb
from object_recognition_core.pipelines.training import TrainingPipeline
from object_recognition_core.utils.find_classes import find_factories
from object_recognition_core.utils.training_detection_args import read_arguments_training

if __name__ == '__main__':
    # read the config file
    source_params, pipeline_params, sink_params, args = read_arguments_training()

    pipelines = find_factories([ pipeline_param['module'] for pipeline_param in pipeline_params.itervalues()],
                               TrainingPipeline) #map of string name to pipeline class

    for _pipeline_id, pipeline_param in pipeline_params.iteritems():
        # make sure object_ids is empty (so that we don't have to deal with the DB
        if 'object_ids' in pipeline_param['parameters']:
            pipeline_param['parameters']['object_ids'] = []
        pipeline = pipelines.get(pipeline_param['type'], False)
        if not pipeline:
            sys.stderr.write('Invalid pipeline name: %s\nMake sure that the pipeline type is defined by a TrainingPipeline class, in the name class function.' % pipeline_param['method'])
            sys.exit(-1)
        object_db = ObjectDb(pipeline_param['parameters'].get('db', {}))
        kwargs = {'object_db':object_db, 'observation_ids':[],
                                         'pipeline_params':pipeline_param.get('parameters', {}),
                                         'subtype':pipeline_param['subtype'],
                                         'args':''}
        processor = pipeline.processor(**kwargs)
        post_processor = pipeline.post_processor(**kwargs)
