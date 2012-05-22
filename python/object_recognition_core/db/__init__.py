# import from the boost wrapped C++ structures. Use a specific name in the imports intead 
# of an import * to make sure of what we have
from object_recognition_core.boost.interface import ObjectDbParameters, ObjectDbTypes, Documents, Models, Document
from .object_db import ObjectDb
