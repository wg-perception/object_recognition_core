'''
Define the TOD detection pipeline.
'''
from object_recognition.pipelines import DetectionPipeline
from argparse import ArgumentParser
from ecto_opencv import highgui, calib
from object_recognition.common.filters.masker import Masker
from object_recognition.common.io.sink import Sink
from object_recognition.common.io.source import Source
from object_recognition.common.utils import json_helper
from object_recognition.common.utils.training_detection_args import read_arguments
from object_recognition.tod.detector import TodDetector, TodDetectorLoader
from ecto_object_recognition.tod_detection import DescriptorLoader
import ecto
import ecto_ros
import sys
from object_recognition import models


class TODDetection(DetectionPipeline):
    def create_pipeline(self, argv=None):
        plasm = ecto.Plasm()
        parser = ArgumentParser()

        # add arguments for the source and sink
        Sink.add_arguments(parser)

        params, args, pipeline_params, do_display, db_params, db = read_arguments(parser, argv)

        model_ids = []
        object_ids = []
        for object_id in params['object_ids']:
            for model_id in models.find_model_for_object(db, object_id, 'TOD'):
                model_ids.append(str(model_id))
                object_ids.append(object_id)
        params['object_ids'] = object_ids

        # TODO handle this properly...
        ecto_ros.init(argv, "tod_detection", False)#not anonymous.

        source = Source.parse_arguments(params['source'])


        sink = Sink.parse_arguments(args, db, db_params, params['object_ids'])

        # define the different pipelines
        for pipeline_param in pipeline_params:
            if pipeline_param['type'] == 'TOD':
                # create the loader and detector
                loader = DescriptorLoader(collection=db_params.collection,
                                           db_params=db_params,
                                           object_ids=object_ids, model_ids=model_ids,
                                           feature_descriptor_params=json_helper.dict_to_cpp_json_str(pipeline_param['feature_descriptor']))
                detector = TodDetector(feature_descriptor_params=pipeline_param['feature_descriptor'],
                                       guess_params=pipeline_param['guess'], search_params=pipeline_param['search'],
                                       display=do_display, rgb_frame_id=params['source']['rgb_frame_id'])
                plasm.connect(loader['descriptors', 'features3d', 'spans', 'id_correspondences', 'do_update'] >>
                              detector['descriptors_db', 'features3d_db', 'spans', 'id_correspondences', 'do_update'])

            # Connect the detector to the source
            for key in source.outputs.iterkeys():
                if key in detector.inputs.keys():
                    plasm.connect(source[key] >> detector[key])

            # define the different outputs
            # TODO, they should all be connected to a merger first
            plasm.connect(detector['object_ids', 'Rs', 'Ts'] >> sink['object_ids', 'Rs', 'Ts'])

        # make sure that we also give the image_message, in case we want to publish a topic
        if 'image_message' in sink.inputs and 'image_message' in source.outputs:
            plasm.connect(source['image_message'] >> sink['image_message'])

        # Display the different poses
        if do_display:
            pose_view = highgui.imshow(name="Pose")
            pose_drawer = calib.PosesDrawer()

            # draw the poses
            plasm.connect(source['image', 'K'] >> pose_drawer['image', 'K'],
                              detector['Rs', 'Ts'] >> pose_drawer['Rs', 'Ts'],
                              pose_drawer['output'] >> pose_view['image']
                              )
        return plasm
