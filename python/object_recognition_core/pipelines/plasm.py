'''
Loaders for all object recognition pipelines
'''
from object_recognition_core.io.sink import Sink
from object_recognition_core.io.source import Source
from object_recognition_core.io.voter import Voter
from object_recognition_core.pipelines.detection import DetectionPipeline, validate_detection_pipeline
from object_recognition_core.pipelines.training import TrainingPipeline
from object_recognition_core.utils.training_detection_args import read_arguments_detector
from object_recognition_core.utils.find_classes import find_classes
import ecto

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
    #map of string name to pipeline class
    pipelines = find_classes([ pipeline_param['package'] for pipeline_param in pipeline_params.itervalues()],
                               DetectionPipeline)
    #map of string name to sink class
    sinks = find_classes([ sink_param.get('package', 'object_recognition_core.io')
                        for sink_param in sink_params.itervalues()], Sink)
    #map of string name to source class
    sources = find_classes([ source_param.get('package', 'object_recognition_core.io')
                          for source_param in source_params.itervalues()], Source)

    # create the different source cells
    source_cells = {}
    for source_id, source_param in source_params.iteritems():
        source_cells[source_id] = sources[source_param['type']].source(**source_param)

    # create the different sink cells
    sink_cells = {}
    for sink_id, sink_param in sink_params.iteritems():
        sink_cells[sink_id] = sinks[sink_param['type']].sink(**sink_param)

    # create the different pipeline cells
    pipeline_cells = {}
    voter_n_input = {}
    for pipeline_id, pipeline_param in pipeline_params.iteritems():
        pipeline = pipelines.get(pipeline_param['method'], False)
        if not pipeline:
            sys.stderr.write('Invalid pipeline name: %s\nMake sure that the pipeline type is defined by a TrainingPipeline class, in the name class function.' % pipeline_param['method'])
            sys.exit(-1)
        pipeline_cells[pipeline_id] = pipeline.detector(**pipeline_param)

        # for each voter id, figure out the number of pipelines connected to it as an input
        for cell_id in pipeline_param.get('voters', []):
            voter_n_input.setdefault(cell_id, 0)
            voter_n_input[cell_id] += 1

    # create the different voter cells
    voter_cells = {}
    for voter_id, voter_param in voter_params.iteritems():
        voter_cells[voter_id] = Voter.create_voter(voter_n_input[voter_id], voter_param)

    # build the plasm with all the pipelines
    plasm = ecto.Plasm()
    for pipeline_id, detector in pipeline_cells.iteritems():
        pipeline_param = pipeline_params[pipeline_id]
        if 'sinks' in pipeline_param or 'voters' in pipeline_param:
            validate_detection_pipeline(detector)

        # link to the different sources
        for source_id in pipeline_param['sources']:
            if source_id in source_cells:
                source = source_cells[source_id]
            else:
                source = pipeline_cells[source_id]
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

    # link the different sources to the sinks
    for sink_id, sink_param in sink_params.iteritems():
        sink = sink_cells[voter_id]
        for source_id in sink_param.get('sources', []):
            plasm = connect_cells(source_cells[source_id], sink, plasm)

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
