#!/usr/bin/env python
"""
Module defining several inputs for the object recognition pipeline
""" 

import ecto
import ecto.opts
from ecto_object_recognition.io import GuessCsvWriter
import sys

########################################################################################################################

class Source(ecto.BlackBox):
    """
    Blackbox that can output to anything
    If a new type of sink is created, add it in the enum list and update the add_arguments and parse_arguments
    """
    ROS_BAG = 'ros_bag'
    ROS_KINECT = 'ros_kinect'

    def __init__(self, plasm):
        """
        sinks: a list of the sinks to use
        """
        ecto.BlackBox.__init__(self, plasm)

        # try to import ecto_ros
        self._do_use_ros = True
        # TODO remove the following line when Blackbox is fully supported
        self._plasm = plasm
        try:
            __import__('ecto_ros')
        except:
            self._do_use_ros = False
        if self._do_use_ros:
            import ecto_ros
            ecto_ros.init(sys.argv, "ecto_node")

        # add the different possible outputs
        self._cell_factory = {}
        self._cells = []
        self._outputs = {}

    # common ecto implementation
    def expose_inputs(self):
        return {}

    def expose_outputs(self):
        return self._outputs

    def expose_parameters(self):
        return {}

    def connections(self):
        return []

    # Functions to help with the argument parsing
    def add_arguments(self, parser):
        if self._do_use_ros:
            from ros.source import BagReader, KinectReader
            parser.add_argument('--ros_bag', dest='do_ros_bag', action='store_true', default = False,
                                help='If set, read from a Kinect controlled by ROS.')
            # TODO
            #self._cell_factory[Source.ROS_BAG] = ecto.opts.cell_options(parser, BagReader, 'ros_bag')

            parser.add_argument('--ros_kinect', dest='do_ros_kinect', action='store_true', default = False,
                                help='If set, read from a ROS bag.')
            # TODO
            #self._cell_factory[ROS_KINECT] = ecto.opts.cell_options(parser, KinectReader, 'ros_kinect')
            self._cell_factory[Source.ROS_KINECT] = KinectReader(self._plasm)

    def parse_arguments(self, parser):
        args = parser.parse_args()
        if args.do_ros_bag:
            cell = self._cell_factory[Source.ROS_BAG](parser)
            self._cells.append(cell)
            self._outputs.update({'image': cell['image'], 'point_cloud': cell['point_cloud']})
        if args.do_ros_kinect:
            #TODO fix the following
            #cell = self._cell_factory[Source.ROS_KINECT](parser)
            cell = self._cell_factory[Source.ROS_KINECT]
            self._cells.append(cell)
            self._outputs.update({'image': cell['image'], 'points3d': cell['points3d'], 'K': cell['K'],
                                  'image_message': cell['image_message']})
