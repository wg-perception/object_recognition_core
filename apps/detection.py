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
from ecto_object_recognition import tod_detection, ros
from object_recognition.tod.feature_descriptor import FeatureDescriptor
from object_recognition.common.io.ros.source import KinectReader, BagReader
from object_recognition.common.io.sink import Sink
from object_recognition.tod.detector import TodDetector

CSV = False
DEBUG = False
DISPLAY = True

PoseArrayPub = ecto_geometry_msgs.Publisher_PoseArray

########################################################################################################################


if __name__ == '__main__':
    plasm = ecto.Plasm()
    sink = Sink(plasm)

    parser = ArgumentParser()
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
    parser.add_argument("-k", "--kinect", dest="do_kinect", help="if set to something, it will read data from the kinect")

    options = parser.parse_args()

    # define the input
    if options.config_file is None or not os.path.exists(options.config_file):
        raise 'option file does not exist'

    # Get the parameters from the file
    json_params = json.loads(str(open(options.config_file).read()))
    feature_descriptor_params = eval(str(json_params['feature_descriptor']).replace("'", '"').replace('u"', '"').\
                                     replace('{u', '{'))
    db_json_params = str(json_params['db']).replace("'", '"').replace('u"', '"').replace('{u', '{')
    object_ids = eval(str(json_params['object_ids']).replace("'", '"').replace('u"', '"').replace('{u', '{'))
    guess_json_params = str(json_params['guess']).replace("'", '"').replace('u"', '"').replace('{u', '{')
    search_json_params = str(json_params['search']).replace("'", '"').replace('u"', '"').replace('{u', '{')

    # define the input
    tod_detector = TodDetector(plasm, feature_descriptor_params, db_json_params, object_ids, search_json_params,
                                 guess_json_params)

    if options.bag:
        bag_reader = BagReader(plasm, dict(image=ecto_sensor_msgs.Bagger_Image(topic_name='image_mono'),
                           camera_info=ecto_sensor_msgs.Bagger_CameraInfo(topic_name='camera_info'),
                           point_cloud=ecto_sensor_msgs.Bagger_PointCloud2(topic_name='points'),
                           ), options.bag)

        # connect to the model computation
        point_cloud_to_mat = tod_detection.PointCloudToMat()
        plasm.connect(bag_reader['image'] >> tod_detector['image'],
                      bag_reader['point_cloud'] >> point_cloud_to_mat['point_cloud'],
                      point_cloud_to_mat['points'] >> tod_detector['points'])

    elif options.do_kinect:
        ecto_ros.init(sys.argv, "ecto_node")
        kinect_reader = KinectReader(plasm, DISPLAY)
        plasm.connect(kinect_reader['image'] >> tod_detector['image'],
                      kinect_reader['points3d'] >> tod_detector['points3d'])

    # write data back to a file
    if CSV:
        guess_writer = tod_detection.GuessCsvWriter()
        plasm.connect(tod_detector['object_ids'] >> guess_writer['object_ids'],
                  tod_detector['Rs'] >> guess_writer['Rs'],
                  tod_detector['Ts'] >> guess_writer['Ts'],
                  )
    #assemble a pose array message
    pose_array_assembler = ros.PoseArrayAssembler()
    #publish the poses over ROS.
    #http://ecto.willowgarage.com/releases/amoeba-beta3/ros/geometry_msgs.html#Publisher_PoseArray
    pose_pub = PoseArrayPub(topic_name='/object_recognition/poses', latched = True)
    plasm.connect(tod_detector['object_ids','Rs','Ts'] >> pose_array_assembler['object_ids','Rs','Ts'],
                  kinect_reader['image_message'] >> pose_array_assembler['image_message'],
                  pose_array_assembler['pose_message'] >> pose_pub[:]
                  )

    # Display the different poses
    if DISPLAY:
        image_view = highgui.imshow(name="RGB", waitKey=1, autoSize=True)
        keypoints_view = highgui.imshow(name="Keypoints", waitKey=1, autoSize=True)
        pose_view = highgui.imshow(name="Pose", waitKey=1, autoSize=True)
        draw_keypoints = features2d.DrawKeypoints()
        pose_drawer = calib.PosesDrawer()
        if options.do_kinect:
            plasm.connect(kinect_reader['image'] >> image_view['input'],
                       kinect_reader['image'] >> draw_keypoints['image'],
                       tod_detector['keypoints'] >> draw_keypoints['keypoints'],
                       draw_keypoints['image'] >> keypoints_view['input']
                       )
            # draw the poses
            plasm.connect(kinect_reader['image'] >> pose_drawer['image'],
                          kinect_reader['K'] >> pose_drawer['K'],
                          tod_detector['Rs'] >> pose_drawer['Rs'],
                          tod_detector['Ts'] >> pose_drawer['Ts'],
                          pose_drawer['output'] >> pose_view['input']
                          )

    # display DEBUG data if needed
    if DEBUG:
        print plasm.viz()
        ecto.view_plasm(plasm)

    # execute the pipeline
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
