'''
Loaders for all object recognition pipelines
'''
from abc import ABCMeta
from ecto import BlackBoxCellInfo as CellInfo
from object_recognition_core.db import Document, Documents, ObjectDb, ObjectDbParameters
from object_recognition_core.db.cells import ObservationReader
import ecto

class TrainerBase(object):
    """
    This is a base class for a training pipeline: you don't need to have your pipeline cell inherit from that class
    but if you do, it will be listed as an official training pipeline
    You need to call the BlackBox constructor in your __init__ first and then this function. Typically, your __init__ is

        >>> class Foo(ecto.BlackBox, TrainerBase):
        >>>     def __init__(self, *args, **kwargs):
        >>>         ecto.BlackBox.__init__(self, *args, **kwargs)
        >>>         TrainerBase.__init__(self)
    """
    pass

class ObservationDealer(ecto.BlackBox):
    '''
    At each iteration, will return one fully typed observation, K,R,T,image,depth,mask, etc...
    Initialized with a predetermined set of observation ids.
    '''
    @staticmethod
    def declare_cells(p):
        return {'db_reader': CellInfo(ObservationReader),
                'observation_dealer': CellInfo(ecto.Dealer, {'tendril': ecto.Tendril(Document()),
                'iterable': [ x for x in Documents(ObjectDb(ObjectDbParameters(p.json_db)), p.observation_ids)]})
               }

    @staticmethod
    def declare_direct_params(p):
        p.declare('observation_ids', 'An iterable of observation ids.', [])
        p.declare('json_db', 'The parameters as a JSON string defining the db to query the parameters from.', '')

    @staticmethod
    def declare_forwards(_p):
        return ({}, {}, {'db_reader': 'all'})

    def connections(self, _p):
        graph = [self.observation_dealer[:] >> self.db_reader['document']]
        return graph
