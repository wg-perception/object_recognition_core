"""
Module defining several voters for the object recognition pipeline
"""

from ecto.load_pybindings import load_pybindings
load_pybindings(__name__)

from object_recognition_core.ecto_cells.voter import Aggregator

class Voter(object):
    '''
    An RGB, Depth Map source.
    '''
    @staticmethod
    def create_voter(n_inputs, voter_params):
        voter_type = voter_params['type']
        if voter_type == 'aggregator':
            return Aggregator(n_inputs=n_inputs)
