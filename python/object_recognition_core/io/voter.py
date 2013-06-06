"""
Module defining several voters for the object recognition pipeline
"""

from object_recognition_core.ecto_cells.voter import Aggregator as AggregatorCpp
import ecto
from ecto.blackbox import BlackBoxCellInfo as CellInfo

########################################################################################################################

class VoterBase(object):
    """
    This is a base class for a voter: you don't need to have your voter cell inherit from that class but if you do,
    it will make sure that its inputs/outputs fit the ORK standard (which is good if you want to interact with
    the official ORK pipelines).
    You need to call the BlackBox constructor in your __init__ first and then this function. Typically, your __init__ is
    class Foo(ecto.BlackBox, VoterBase):
        def __init__(self, *args, **kwargs):
            ecto.BlackBox.__init__(self, *args, **kwargs)
            VoterBase.__init__(self)
    """

    def __init__(self):
        """
        This ensures that the given cell exhibits the minimal interface to be
        considered a voter for object recognition
        """
        pass

########################################################################################################################

class Aggregator(ecto.BlackBox, VoterBase):
    """
    Cell meant to take several outputs from pipelines and aggregate the results
    """
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)
        VoterBase.__init__(self)

    @staticmethod
    def declare_cells(_p):
        return {'main': CellInfo(AggregatorCpp)}

    @staticmethod
    def declare_forwards(_p):
        return ({'main': 'all'}, {'main': 'all'}, {'main': 'all'})

    def connections(self, _p):
        return [self.main]
