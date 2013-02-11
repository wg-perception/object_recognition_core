"""
Module defining a common Python interface to an ObjectDb.
It also provides a factory you can use to wrap your own DB
"""

from abc import ABCMeta
from object_recognition_core.boost.interface import ObjectDbParameters
from object_recognition_core.utils.find_classes import find_classes
import json

########################################################################################################################

class ObjectDbFactory(object):
    """
A base class for a factory that can allow you to wrap your own ObjectDb
"""

    __metaclass__ = ABCMeta

    @classmethod #see http://docs.python.org/library/abc.html#abc.ABCMeta.__subclasshook__
    def __subclasshook__(cls, C):
        if C is ObjectDb:
            #all pipelines must have atleast this function.
            if any("type_name" in B.__dict__ for B in C.__mro__):
                return True
        return NotImplemented

    @classmethod
    def type_name(cls):
        """
Return the code name for your ObjectDb
"""
        raise NotImplementedError("The ObjectDb type_name function must return a string name.")

    @classmethod
    def object_db(cls, db_params):
        """
Return the ObjectDbBase object
:param db_params: an object of type ObjectDbParameters that you can use to initiate your DB
"""
        raise NotImplementedError("The ObjectDb object_db function must return a C++ wrapped ObjectDb.")

########################################################################################################################

def core_db_types():
    """
Return the current DB types implemented in object_recognition_core
:returns: a list of string matching the ObjectDb types
"""
    types = []
    from object_recognition_core.db import ObjectDbTypes
    for db_type in ObjectDbTypes.values.values():
        types.append(str(db_type).split('.')[-1].lower())
    types.remove('noncore')
    return types

def ObjectDb(db_params):
    """
    Returns the ObjectDb for the given db_params given as a dictionary
    It crawls the object_recognition_core module or any other module
    in order to find the ObjectDb you are looking for

    :param db_params: ObjectDbParameters defining a DB, or json string or dict
    """

    if (isinstance(db_params, ObjectDbParameters)):
        db_params_raw = db_params.raw
        object_db_params = db_params
    elif (isinstance(db_params, str)):
        db_params_raw = json.loads(db_params)
        object_db_params = ObjectDbParameters(db_params)
    else:
        db_params_raw = db_params
        object_db_params = ObjectDbParameters(db_params)

    # check if it is a conventional DB from object_recognition_core
    db_type = db_params_raw.get('type', None)
    if db_type.lower() in core_db_types():
        from object_recognition_core.boost.interface import ObjectDb as ObjectDbCpp
        return ObjectDbCpp(object_db_params)

    # otherwise, look for the possible modules for that DB type
    module = db_params_raw.get('module', None)
    if not module:
        raise RuntimeError("The 'module' property is not set. It is required to find the DB object")
    for db_factory in find_classes([module], [ObjectDbFactory]):
        if db_factory.__name__ == db_type:
            return db_factory.object_db(db_params_raw)
