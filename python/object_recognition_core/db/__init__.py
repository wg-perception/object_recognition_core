"""
When building the db boost python module, a db.so file is created.
During install time, everything is put in lib/python*/dist-packages/object_recognition_core/db and all is fine
(the db.so and the */py files are in the same directory).
But at build time, db.so is in the build/lib/object_recognition_core/db/ folder which means it is not found
"""

from object_recognition_core.utils.load_pybindings import load_pybindings
load_pybindings(__name__)
