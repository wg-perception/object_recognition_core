"""
Module defining a common Python interface to an ObjectDb
"""

from abc import ABCMeta
from image_pipeline.io.source import create_source
from object_recognition_core.db.interface import ObjectDb as ObjectDbCpp, ObjectDbParameters

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
        raise NotImplementedError("The ObjectDb class must return a string name.")

    @classmethod
    def object_db(cls, db_params):
        """
        Return the ObjectDbBase object
        :db_params a dictionary of the parameters to use
        """
        raise NotImplementedError("The ObjectDb class must return a string name.")

########################################################################################################################

def ObjectDb(db_params):
    """
    Returns the ObjectDb for the given db_params given as a dictionary
    """
    def _core_types():
        """\
        Return the current DB types implemented in object_recognition_core
        """
        types = []
        from object_recognition_core.db.interface import db_types as db_types
        for type in db_types.values.itervalues():
            types.append(str(type).split('.')[-1].lower())
        return types

    # check if it is a conventional DB from object_recognition_core
    type = db_params.get('type', None)
    if type.lower() in _core_types():
        return ObjectDbCpp(ObjectDbParameters(db_params))

    # otherwise, look for the possible modules for that DB type
    module = db_params.get('module', None)
    if not module:
        raise RuntimeError("The 'module' property is not set. It is required to find the DB object")
    object_db_bases = find_cells(module, ObjectDbBase)
    if type not in object_db_bases:
        raise RuntimeError('The db type %s was not found in module %s' % (type, module))
    return ObjectDbBase.object_db(db_params)
