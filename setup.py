#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()

d['packages'] = ['object_recognition_core', 'object_recognition_core.db', 'object_recognition_core.filters',
                'object_recognition_core.io', 'object_recognition_core.pipelines',
                'object_recognition_core.utils', 'couchdb']
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
