#!/usr/bin/env python
from distutils.core import setup
db  filters  __init__.py  io  pipelines  utils

setup(name='Object recognition core',
      version='1.0.0',
      description='The core of object recognition',
      packages=['object_recognition_core', 'object_recognition_core.db', 'object_recognition_core.filters',
                'object_recognition_core.io', 'object_recognition_core.io.ros', 'object_recognition_core.io.ros.source',
                'object_recognition_core.io.ros.sink', 'object_recognition_core.pipelines',
                'object_recognition_core.utils', 'couchdb'],
      package_dir={'':'python'}
)
