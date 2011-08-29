#!/usr/bin/env python
"""
Module defining a function that returns the appropriate ecto cells for Feature and Descriptor finding
"""

from ecto_opencv import features2d

def FeatureDescriptor(feature_descriptor_params):
    """
    Function that takes JSON parameters for Feature/Descriptor extraction and that returns the appropriate cell
    It has the following keys:
    'combination':
        'type': can be 'ORB' or 
    """
    print feature_descriptor_params
    if feature_descriptor_params.has_key('combination'):
        if feature_descriptor_params['combination']['type'] == 'ORB':
            params = feature_descriptor_params.get('feature', {})
            params.update(feature_descriptor_params.get('descriptor', {}))
            return features2d.ORB(**params)
