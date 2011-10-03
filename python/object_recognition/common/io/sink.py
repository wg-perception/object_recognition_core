#!/usr/bin/env python
"""
Module defining several outputs for the object recognition pipeline
""" 

import ecto
import ecto.opts
from ecto_object_recognition.io import GuessCsvWriter
import sys

########################################################################################################################

class Sink(ecto.BlackBox):
    """
    Blackbox that can output to anything
    If a new type of sink is created, add it in the enum list and update the add_arguments and parse_arguments
    """
    CSV_WRITER = 'csv_writer'
    ROS = 'ros'
    ROS_TABLETOP = 'ros_tabletop'

    def __init__(self, plasm):
        """
        sinks: a list of the sinks to use
        """
        ecto.BlackBox.__init__(self, plasm)

        self._plasm = plasm

        self._object_ids_splitter = ecto.Passthrough()
        self._Rs_splitter = ecto.Passthrough()
        self._Ts_splitter = ecto.Passthrough()

        # try to import ecto_ros
        self._do_use_ros = True
        try:
            __import__('ecto_ros')
        except:
            self._do_use_ros = False

        if self._do_use_ros:
            import ecto_ros
            ecto_ros.init(sys.argv, "ecto_node")
            self._image_message_splitter = ecto.Passthrough()

        # add the different possible outputs
        self._cell_factory = {}
        self._cells = []

    # common ecto implementation
    def expose_inputs(self):
        inputs = {'object_ids':self._object_ids_splitter['in'],
                'Rs':self._Rs_splitter['in'],
                'Ts':self._Ts_splitter['in']}
        if self._do_use_ros:
            inputs['image_message'] = self._image_message_splitter['in']
        return inputs

    def expose_outputs(self):
        return {}

    def expose_parameters(self):
        return {}

    def connections(self):
        connections = []
        for cell in self._cells:
            connections.extend([self._object_ids_splitter['out'] >> cell['object_ids'],
                                self._Rs_splitter['out'] >> cell['Rs'],
                                self._Ts_splitter['out'] >> cell['Ts']])
            if self._do_use_ros:
                connections.append(self._image_message_splitter['out'] >> cell['image_message'])
        return connections

    # Functions to help with the argument parsing
    def add_arguments(self, parser):
        parser.add_argument('--csv', dest='do_csv', action='store_true', default = False,
                            help='If set, output to a CSV NIST file')
        self._cell_factory[Sink.CSV_WRITER] = ecto.opts.cell_options(parser, GuessCsvWriter, 'csv')

        if self._do_use_ros:
            from ros.sink import Publisher, TabletopPublisher
            parser.add_argument('--ros_topic', dest='ros_topic', help='If set, publish to a ROS topic')
            # TODO
            #self._cell_factory[Sink.ROS] = ecto.opts.cell_options(parser, Publisher, 'ros')
    
            parser.add_argument('--ros_tabletop_topic', dest='ros_tabletop_topic',
                                help='If set, publish to this ROS topic in the tabletop format')
            # TODO
            #self._cell_factory[Sink.ROS_TABLETOP] = ecto.opts.cell_options(parser, TabletopPublisher, 'ros tabletop')

    def parse_arguments(self, parser):
        args = parser.parse_args()
        if args.do_csv:
            self._cells.append(self._cell_factory[Sink.CSV_WRITER](args))
        if self._do_use_ros:
            if args.ros_topic:
                #TODO, use the following when working
                #self._cells.append(self._cell_factory[Sink.ROS](parser))
                from ros.sink import Publisher
                self._cells.append(Publisher(self._plasm, 'poses', 'object_ids', True))
            if args.ros_tabletop_topic:
                self._cells.append(self._cell_factory[Sink.ROS_TABLETOP](args))
