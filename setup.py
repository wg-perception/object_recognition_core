#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['object_recognition_core', 'object_recognition_core.db', 'object_recognition_core.filters',
                'object_recognition_core.io', 'object_recognition_core.pipelines',
                'object_recognition_core.utils', 'couchdb']
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
