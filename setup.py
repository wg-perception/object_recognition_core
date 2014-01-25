#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['object_recognition_core', 'object_recognition_core.db', 'object_recognition_core.filters',
                'object_recognition_core.io', 'object_recognition_core.pipelines',
                'object_recognition_core.utils', 'couchdb'],
    package_dir={'': 'python'},
    scripts=['apps/detection', 'apps/training', 'apps/dbscripts/copy_db.py',
             'apps/dbscripts/mesh_add.py', 'apps/dbscripts/object_add.py',
             'apps/dbscripts/object_delete.py', 'apps/dbscripts/object_search.py']
)

setup(**d)
