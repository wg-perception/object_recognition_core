#!/usr/bin/env python
"""
Module defining a function that returns the appropriate ecto cells for Feature and Descriptor finding
"""

import ecto
from ecto_opencv import features2d

class FeatureDescriptor(ecto.BlackBox):
    """
    Function that takes JSON parameters for Feature/Descriptor extraction and that returns the appropriate cell
    It has the following keys:
    'combination':
        'type': can be 'ORB' or 
    """
    ORB_combination = features2d.ORB
    def declare_params(self, p):
        p.declare('json_params', 'JSON parameters for the features/descriptors as a a string.', '{}')

    def declare_io(self, p, i, o):
        params = eval(p.json_params)
        #self._json_params = json_params
        if params:
            if params.has_key('combination'):
                if params['combination'] == 'ORB':
                    self._feature_descriptor_params = params.get('feature', {})
                    self._feature_descriptor_params.update(params.get('descriptor', {}))
                    self._cell_name = 'ORB_combination'
                else:
                    raise 'parameters not supported for FeatureDescriptor'
            else:
                raise 'parameters not supported for FeatureDescriptor'
        else:
            self._feature_descriptor_params = {'n_features':1000}
            self._cell_name = 'ORB_combination'
        self._feature_descriptor_params = {'n_features':1000}
        self._cell_name = 'ORB_combination'

        i.forward('image', cell_name = self._cell_name, cell_key = 'image')
        i.forward('mask', cell_name = self._cell_name, cell_key = 'mask')
        o.forward('keypoints', cell_name = self._cell_name, cell_key = 'keypoints')
        o.forward('descriptors', cell_name = self._cell_name, cell_key = 'descriptors')

    def configure(self, p, i, o):
        if self._cell_name == 'ORB_combination':
            #print self._feature_descriptor_params
            self.ORB_combination = self.ORB_combination(**self._feature_descriptor_params)
            self.cell = self.ORB_combination

    def connections(self):
        if self._feature_descriptor_params.has_key('combination'):
            return [ self.cell ]
        else:
            return [ self.cell ]
