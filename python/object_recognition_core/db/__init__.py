"""
When building the db boost python module, a db.so file is created.
During install time, everything is put in lib/python*/dist-packages/obejct_recognition_core/db and all is fine
(the db.so and the */py files are in the same directory).
But at build time, db.so is in the build/lib/object_recognition_core/db/ folder which means it is not found
"""

import os

# go over the PYTHONPATH, and add any folder ending in object_recognition_core/db
INSTALL_FOLDER = os.path.join('object_recognition_core', 'db')

for path in os.environ['PYTHONPATH'].split(':'):
    potential_additional_path = os.path.join(path, INSTALL_FOLDER)
    if os.path.isdir(potential_additional_path) and path not in __path__:
        __path__.append(potential_additional_path)
