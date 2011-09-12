#!/usr/bin/env python
"""
Module defining a function that returns the appropriate ecto cells for Feature and Descriptor finding
"""

from ecto_opencv import features2d

def FeatureDescriptor(json_params):
    """
    Function that takes JSON parameters for Feature/Descriptor extraction and that returns the appropriate cell
    It has the following keys:
    'combination':
        'type': can be 'ORB' or 
    """
    if json_params.has_key('combination'):
        if json_params['combination']['type'] == 'ORB':
            params = json_params.get('feature', {})
            params.update(json_params.get('descriptor', {}))
            return features2d.ORB(**params)
        else:
            raise 'parameters not supported for FeatureDescriptor'
    else:
        raise 'parameters not supported for FeatureDescriptor'
