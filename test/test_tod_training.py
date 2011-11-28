#!/usr/bin/env python

import ecto
from ecto_object_recognition import capture
from object_recognition.tod.trainer import TODTrainingPipeline
from object_recognition.common.utils.parser import ObjectRecognitionParser

T = TODTrainingPipeline()

parser = ObjectRecognitionParser()

args = parser.parse_args()

T.incremental_model_builder({'feature_descriptor': {'type': 'ORB'}}, args)
T.post_processor({'search': {'type': 'LSH'}}, args)
