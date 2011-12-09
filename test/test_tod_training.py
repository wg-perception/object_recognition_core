#!/usr/bin/env python

import ecto
from object_recognition.tod.trainer import TODTrainingPipeline
from object_recognition.common.utils.parser import ObjectRecognitionParser

T = TODTrainingPipeline()

parser = ObjectRecognitionParser()

args = parser.parse_args()

submethod = {'descriptor': {'type':'ORB'}}
parameters = {'feature': {'type': 'ORB'}}
T.incremental_model_builder(submethod=submethod, pipeline_params=parameters, args=args)
T.post_processor(submethod=submethod, pipeline_params={'search': {'type': 'LSH'}}, args=args)
