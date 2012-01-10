'''
Loaders for all object recognition pipelines
'''
from abc import ABCMeta, abstractmethod
from object_recognition.common.io.sink import Sink
from object_recognition.common.io.source import Source
from object_recognition.common.io.voter import Voter
from object_recognition.common.utils.training_detection_args import read_arguments_detector
from object_recognition.pipelines.detection import DetectionPipeline
from object_recognition.pipelines.training import TrainingPipeline
import ecto
import inspect
import pkgutil
import sys

def find_pipelines(modules, pipeline_type):
    '''
    Given a list of python packages, or modules, find all TrainingPipeline implementations.
    :param modules: The names of the modules to look into
    :returns: A list of TrainingPipeline implementation classes.
    '''
    pipelines = {}
    ms = []
    for module in modules:
        m = __import__(module)
        ms += [m]
        for loader, module_name, is_pkg in  pkgutil.walk_packages(m.__path__):
            if is_pkg:
                module = loader.find_module(module_name).load_module(module_name)
                ms.append(module)
    for pymodule in ms:
        for x in dir(pymodule):
            potential_pipeline = getattr(pymodule, x)
            if inspect.isclass(potential_pipeline) and potential_pipeline != pipeline_type and \
                                                            issubclass(potential_pipeline, pipeline_type):
                pipelines[potential_pipeline.type_name()] = potential_pipeline
    return pipelines

def connect_cells(cell1, cell2, plasm):
    """
    Given two cells, connect them with all the possible tendrils in the plasm
    """
    for key in set(cell1.outputs.keys()).intersection(cell2.inputs.keys()):
        plasm.connect(cell1[key] >> cell2[key])
    return plasm

def create_detection_plasm():
    """
    Function that returns the detection plasm corresponding to the input arguments
    """
    source_params, pipeline_params, sink_params, voter_params, args = read_arguments_detector()
    pipelines = find_pipelines([ pipeline_param['package'] for pipeline_param in pipeline_params.itervalues()],
                               DetectionPipeline) #map of string name to pipeline class

    # create the different source cells
    source_cells = {}
    for source_id, source_param in source_params.iteritems():
        source_cells[source_id] = Source.parse_arguments(source_param)
    # create the different sink cells
    sink_cells = {}
    for sink_id, sink_param in sink_params.iteritems():
        sink_cells[sink_id] = Sink.parse_arguments(sink_param)

    # for each voter id, figure out the number of pipelines connected to it as an input
    voter_n_input = {}
    for pipeline_param in pipeline_params.itervalues():
        for cell_id in pipeline_param.get('voters', []):
            voter_n_input.setdefault(cell_id, 0)
            voter_n_input[cell_id] += 1

    # create the different voter cells
    voter_cells = {}
    for voter_id, voter_param in voter_params.iteritems():
        voter_cells[voter_id] = Voter.create_voter(voter_n_input[voter_id], voter_param)

    # build the plasm with all the pipelines
    plasm = ecto.Plasm()
    for _pipeline_id, pipeline_param in pipeline_params.iteritems():
        pipeline = pipelines.get(pipeline_param['method'], False)
        if not pipeline:
            sys.stderr.write('Invalid pipeline name: %s\nMake sure that the pipeline type is defined by a TrainingPipeline class, in the name class function.' % pipeline_param['method'])
            sys.exit(-1)
        detector = pipeline().detector(**pipeline_param)
        if 'sinks' in pipeline_param or 'voters' in pipeline_param:
            pipeline.validate(detector)

        # link to the different sources
        for source_id in pipeline_param['sources']:
            source = source_cells[source_id]
            plasm = connect_cells(source, detector, plasm)
        # link to the different sink
        for sink_id in pipeline_param.get('sinks', []):
            sink = sink_cells[sink_id]
            plasm = connect_cells(detector, sink, plasm)

        # link to the different voters
        for voter_id in pipeline_param.get('voters', []):
            voter = voter_cells[voter_id]
            # connect the common tendrils
            plasm = connect_cells(detector, voter, plasm)
            # connect the pose_results tendrils
            plasm.connect(detector['pose_results'] >> voter['pose_results%d' % voter_n_input[voter_id] ])
            voter_n_input[voter_id] -= 1

    # link the different voters to the sinks
    for voter_id, voter_param in voter_params.iteritems():
        voter = voter_cells[voter_id]
        for sink_id in voter_param.get('sinks', []):
            plasm = connect_cells(voter, sink_cells[sink_id], plasm)

    # ROS specific
    # make sure that we also give the image_message, in case we want to publish a topic
    for source in source_cells.itervalues():
        for sink in sink_cells.itervalues():
            if 'image_message' in sink.inputs and 'image_message' in source.outputs:
                plasm.connect(source['image_message'] >> sink['image_message'])

    return plasm
