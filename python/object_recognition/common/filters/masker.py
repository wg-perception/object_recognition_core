#!/usr/bin/env python
"""
Module defining several inputs for the object recognition pipeline
"""

import ecto
import ecto.opts
import sys

########################################################################################################################

class Masker(ecto.BlackBox):
    """
    Blackbox that masks out certain areas of the image
    If a new type of masker is created, add it in the enum list and update the add_arguments and parse_arguments
    """
    
    #TODO FIXME
    DEPTH = 'depth'

    def __init__(self, plasm):
        """
        sinks: a list of the sinks to use
        """
        ecto.BlackBox.__init__(self, plasm)

        # add the different possible outputs
        self._cell_factory = {}
        self._cells = []

    # common ecto implementation
    def declare_io(self, p, i ,o):
        if self._cells:
            return {'points3d': self._cells[0]['points3d']}
        else:
            return {}

    def expose_outputs(self):
        if self._cells:
            return {'mask': self._cells[-1]['mask']}
        else:
            return {}

    def expose_parameters(self):
        return {}

    def connections(self):
        return []

    # Functions to help with the argument parsing
    def add_arguments(self, parser):
        from ecto_object_recognition.common.filters import DepthFilter
        self._cell_factory[Masker.DEPTH] = ecto.opts.cell_options(parser, DepthFilter, 'depth_filter')

    def parse_arguments(self, parser):
        args = parser.parse_args()
        if hasattr(args, 'd_min') or hasattr(args, 'd_max'):
            cell = self._cell_factory[Source.ROS_BAG](parser)
            self._cells.append(cell)
