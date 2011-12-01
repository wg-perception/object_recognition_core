#!/usr/bin/env python

import ecto
from ecto_object_recognition import capture
from object_recognition.tod.trainer import TODTrainingPipeline
from object_recognition.common.utils.parser import ObjectRecognitionParser

T = TODTrainingPipeline()

parser = ObjectRecognitionParser()

args = parser.parse_args()

submethod = {'descriptor': {'type':'ORB'}}
parameters = {'feature': {'type': 'ORB'}}
T.incremental_model_builder(submethod, parameters, args)
T.post_processor(submethod, {'search': {'type': 'LSH'}}, args)
