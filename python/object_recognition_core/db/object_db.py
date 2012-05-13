"""
Module defining a common Python interface to an ObjectDb
"""

from abc import ABCMeta
from ecto_image_pipeline.io.source import create_source
from object_recognition_core.boost.interface import ObjectDbParameters
from object_recognition_core.utils.find_classes import find_classes

########################################################################################################################

class ObjectDbBase(object):
    '''
    An ObjectDb abstract base class
    '''

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
        :db_params a dictionary of the parameters to use
        """
        raise NotImplementedError("The ObjectDb object_db function must return a C++ wrapped ObjectDb.")

########################################################################################################################

def core_db_types():
    """
    Return the current DB types implemented in object_recognition_core
    """
    types = []
    from object_recognition_core.db import DbTypes
    for type in DbTypes.values.itervalues():
        types.append(str(type).split('.')[-1].lower())
    types.remove('noncore')
    return types

def ObjectDb(db_params):
    """
    Returns the ObjectDb for the given db_params given as a dictionary
    """

    if (isinstance(db_params, ObjectDbParameters)):
        db_params_raw = db_params.raw
        object_db_params = db_params
    else:
        db_params_raw = db_params
        object_db_params = ObjectDbParameters(db_params)

    # check if it is a conventional DB from object_recognition_core
    type = db_params_raw.get('type', None)
    if type.lower() in core_db_types():
        from object_recognition_core.boost.interface import ObjectDb as ObjectDbCpp
        return ObjectDbCpp(object_db_params)

    # otherwise, look for the possible modules for that DB type
    module = db_params_raw.get('module', None)
    if not module:
        raise RuntimeError("The 'module' property is not set. It is required to find the DB object")
    object_db_bases = find_classes([module], ObjectDbBase)
    if type not in object_db_bases:
        raise RuntimeError('The db type %s was not found in module %s' % (type, module))
    return object_db_bases[type].object_db(db_params_raw)
