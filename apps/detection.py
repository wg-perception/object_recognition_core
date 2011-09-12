#!/usr/bin/env python
import ecto
import ecto_geometry_msgs
import ecto_pcl
import ecto_ros
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, features2d
import json
from argparse import ArgumentParser
import os
import sys
import time
from ecto_object_recognition import tod_detection
from object_recognition.tod.feature_descriptor import FeatureDescriptor
from object_recognition.common.io.ros.source import KinectReader, BagReader
from object_recognition.common.filters.masker import Masker
from object_recognition.common.io.sink import Sink
from object_recognition.common.io.source import Source
from object_recognition.tod.detector import TodDetector

DEBUG = False
DISPLAY = True

PoseArrayPub = ecto_geometry_msgs.Publisher_PoseArray

########################################################################################################################

if __name__ == '__main__':
    plasm = ecto.Plasm()
    source = Source(plasm)
    masker = Masker(plasm)
    sink = Sink(plasm)

    parser = ArgumentParser()
    # add arguments for the source
    source.add_arguments(parser)

    # add arguments for the sink
    sink.add_arguments(parser)

    parser.add_argument("-b", "--bag", dest="bag", help="The bag to analyze")
    parser.add_argument("-c", "--config_file", dest="config_file",
                      help='the file containing the configuration as JSON. It should contain the following fields.\n'
                      '"feature_descriptor": with parameters for "combination", "feature" and "descriptor".\n'
                      '"db": parameters about the db: "type", "url".\n'
                      '"objects_ids": the list of object to process, e.g. ["amys_country_cheddar_bowl",'
                      '"band_aid_plastic_strips"]\n'
                      '"search": the "type" of the search structure, the "radius" and/or "ratio" for the ratio test.\n'
                      )

    # parse the arguments
    source.parse_arguments(parser)
    sink.parse_arguments(parser)
    args = parser.parse_args()
    
    #todo handle this properly...
    ecto_ros.init(sys.argv, "ecto_node")

    if not source.expose_outputs():
        raise 'no input specified'

    # define the input
    if args.config_file is None or not os.path.exists(args.config_file):
        raise 'option file does not exist'

    # Get the parameters from the file
    json_params = json.loads(str(open(args.config_file).read()))
    json_params = eval(str(json_params['feature_descriptor']).replace("'", '"').replace('u"', '"').\
                                     replace('{u', '{'))

    # define the main cell
    tod_detector = TodDetector(plasm, json_params['tod'], object_ids)

    # define the input
    if 0:
        bag_reader = BagReader(plasm, dict(image=ecto_sensor_msgs.Bagger_Image(topic_name='image_mono'),
                           camera_info=ecto_sensor_msgs.Bagger_CameraInfo(topic_name='camera_info'),
                           point_cloud=ecto_sensor_msgs.Bagger_PointCloud2(topic_name='points'),
                           ), options.bag)

        # connect to the model computation
        point_cloud_to_mat = tod_detection.PointCloudToMat()
        plasm.connect(bag_reader['image'] >> tod_detector['image'],
                      bag_reader['point_cloud'] >> point_cloud_to_mat['point_cloud'],
                      point_cloud_to_mat['points'] >> tod_detector['points'])

    # Define the source
    for key in source.expose_outputs().iterkeys():
        if key in tod_detector.expose_inputs().keys():
            plasm.connect(source[key] >> tod_detector[key])

    # define the different outputs
    plasm.connect(tod_detector['object_ids','Rs','Ts'] >> sink['object_ids','Rs','Ts'])

    # make sure that we also give the image_message, in case we want to publish a topic
    if 'image_message' in sink.expose_inputs() and 'image_message' in source.expose_outputs():
        plasm.connect(source['image_message'] >> sink['image_message'])

    # Display the different poses
    if DISPLAY:
        image_view = highgui.imshow(name="RGB")
        keypoints_view = highgui.imshow(name="Keypoints")
        pose_view = highgui.imshow(name="Pose")
        draw_keypoints = features2d.DrawKeypoints()
        pose_drawer = calib.PosesDrawer()

        plasm.connect(source['image'] >> image_view['input'],
                       source['image'] >> draw_keypoints['image'],
                       tod_detector['keypoints'] >> draw_keypoints['keypoints'],
                       draw_keypoints['image'] >> keypoints_view['input']
                       )
        # draw the poses
        plasm.connect(source['image', 'K'] >> pose_drawer['image', 'K'],
                          tod_detector['Rs', 'Ts'] >> pose_drawer['Rs', 'Ts'],
                          pose_drawer['output'] >> pose_view['input']
                          )

    # display DEBUG data if needed
    if DEBUG:
        print plasm.viz()
        ecto.view_plasm(plasm)

    # execute the pipeline
    sched = ecto.schedulers.Singlethreaded(plasm)
    #sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()
