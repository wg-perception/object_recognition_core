#!/usr/bin/env python
"""
This script tests whether a detection pipeline can be created given a config file.
It is not meant to be run as a test of object_recognition but as a test for and by each 
pipeline independently.
"""

from object_recognition.common.utils.training_detection_args import read_arguments_training
from ecto_object_recognition.object_recognition_db import ObjectDbParameters
from object_recognition.pipelines import find_pipelines
from object_recognition.pipelines.training import TrainingPipeline

if __name__ == '__main__':
    source_params, pipeline_params, sink_params, args = read_arguments_training()
    pipelines = find_pipelines([ pipeline_param['package'] for pipeline_param in pipeline_params.itervalues()],
                               TrainingPipeline) #map of string name to pipeline class

    for _pipeline_id, pipeline_param in pipeline_params.iteritems():
        # make sure object_ids is empty (so that we don't have to deal with the DB
        pipeline_param['parameters']['object_ids'] = []
        pipeline = pipelines.get(pipeline_param['method'], False)
        if not pipeline:
            sys.stderr.write('Invalid pipeline name: %s\nMake sure that the pipeline type is defined by a TrainingPipeline class, in the name class function.' % pipeline_param['method'])
            sys.exit(-1)
        db_params = ObjectDbParameters(pipeline_param['parameters'].get('db', {}))
        kwargs = {'db_params':db_params, 'observation_ids':[],
                                         'pipeline_params':pipeline_param.get('parameters', {}),
                                         'submethod':pipeline_param['submethod'],
                                         'args':''}
        processor = pipeline().processor(**kwargs)
        post_processor = pipeline().post_processor(**kwargs)
