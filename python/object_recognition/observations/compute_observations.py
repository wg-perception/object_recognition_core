#!/usr/bin/env python
import sys
import argparse
import time
import tempfile
import os

import couchdb

import ecto
from ecto_opencv import highgui, calib, imgproc
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs

import object_recognition
from object_recognition import dbtools, models, capture, observations

ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

class CalcObservations(ecto.BlackBox):
    def __init__(self, plasm):
        ecto.BlackBox.__init__(self, plasm)
        self.gray_image = ecto.Passthrough('gray Input')
        self.depth_image = ecto.Passthrough('depth Input')
        self.camera_info = ecto.Passthrough('K')
        self.pose_calc = observations.OpposingDotPoseEstimator(plasm,
                                        rows=5, cols=3,
                                        pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                        square_size=0.04, debug=False)
        self.masker = calib.PlanarSegmentation(z_min=0.02, y_crop=0.10, x_crop=0.10)
        self.delta_pose = capture.DeltaRT("delta R|T", angle_thresh=3.14 / 36) #5 degree increments.

    def expose_outputs(self):
        return {
                'R': self.pose_calc['R'],
                'T': self.pose_calc['T'],
                'mask':self.masker['mask'],
                'novel': self.delta_pose['novel'],
               }

    def expose_inputs(self):
        return {
                'image': self.gray_image[:],
                'depth': self.depth_image[:],
                'K': self.camera_info[:]
               }

    def expose_parameters(self):
        return { }

    def connections(self):
        graph = [
                  self.gray_image[:] >> self.pose_calc['image'],
                  self.camera_info[:] >> (self.pose_calc['K'], self.masker['K']),
                  self.depth_image[:] >> self.masker['depth'],
                  self.pose_calc['R', 'T', 'found'] >> self.delta_pose['R', 'T', 'found'],
                  self.pose_calc['R', 'T'] >> self.masker['R', 'T'],
                ]
        return graph


def connect_observation_calc(sync, commit, object_id, session_id, debug=False):
    plasm = ecto.Plasm()
    depth_ci = ecto_ros.CameraInfo2Cv('camera_info -> cv::Mat')
    image_ci = ecto_ros.CameraInfo2Cv('camera_info -> cv::Mat')

    image = ecto_ros.Image2Mat()
    depth = ecto_ros.Image2Mat()
    #conversions
    plasm.connect(
                  sync["image"] >> image["image"],
                  sync["depth"] >> depth['image'],
                  sync['image_ci'] >> image_ci[:],
                  sync['depth_ci'] >> depth_ci[:]
                  )

    rgb = imgproc.cvtColor('bgr -> rgb', flag=imgproc.Conversion.BGR2RGB)
    gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.Conversion.RGB2GRAY)

    calc_observations = CalcObservations(plasm)
    plasm.connect(image[:] >> (rgb[:], gray[:]),
                  gray[:] >> calc_observations['image'],
                  depth[:] >> calc_observations['depth'],
                  image_ci['K'] >> calc_observations['K']
                  )
    if debug:
        image_display = highgui.imshow('image display', name='image', waitKey=10, autoSize=True)
        mask_display = highgui.imshow('mask display', name='mask', waitKey= -1, autoSize=True)
        plasm.connect(rgb[:] >> image_display[:])
        plasm.connect(calc_observations['mask'] >> mask_display[:])


    if commit:
        db_inserter = capture.ObservationInserter("db_inserter", object_id=object_id, session_id=session_id)
        plasm.connect(depth[:] >> db_inserter['depth'],
                  calc_observations['R', 'T', 'mask', 'novel'] >> db_inserter['R', 'T', 'mask', 'found'],
                  rgb[:] >> db_inserter['image'],
                  image_ci['K'] >> db_inserter['K'],
                  )
    return plasm


def parse_args():
    parser = argparse.ArgumentParser(description='Computes observations from a raw bag of appropriate data.' +
                                     '  This assumes that the bag is already in the database associated with an object.')
    parser.add_argument('-i', '--bag_id', metavar='BAG_ID', dest='bag_id', type=str, default='',
                       help='The id of the bag in DB to compute observations from.')
    parser.add_argument('--all', dest='compute_all', action='store_const',
                        const=True, default=False,
                        help='Compute all observations possible given all bags in the system.')
    parser.add_argument('--visualize', dest='visualize', action='store_const',
                        const=True, default=False,
                        help='Turn on visiualization')
    object_recognition.dbtools.add_db_options(parser)
    args = parser.parse_args()
    return args

def compute_for_bag(bag, bags, args):
    print "Loading bag with id:", bag.id
    bag_file_lo = bags.get_attachment(bag, 'data.bag', None)
    if bag_file_lo == None:
        print "Could not load the attachment:", 'data.bag', 'from bag document:\n', bag
        sys.exit(-1)
    tmp_file = tempfile.NamedTemporaryFile(delete=False)
    try:
        tmp_file.write(bag_file_lo.read())
        tmp_file.close()
        print "Wrote bag to:", tmp_file.name

        baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                        depth=ImageBagger(topic_name='/camera/depth/image'),
                        image_ci=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
                        depth_ci=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
                       )

        sync = ecto_ros.BagReader('Bag Reader',
                                  baggers=baggers,
                                  bag=tmp_file.name,
                                 )

        sessions = dbs['sessions']
        session = models.Session()
        session.object_id = bag.object_id
        session.bag_id = bag.id
        if args.commit:
            session.store(sessions)
        print "running graph"
        plasm = connect_observation_calc(sync, args.commit, str(session.object_id), str(session.id), args.visualize)
        sched = ecto.schedulers.Threadpool(plasm)
        sched.execute()
        return session
    finally:
        print "Removing tmp_file:", tmp_file.name
        os.remove(tmp_file.name)

if "__main__" == __name__:
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    dbs = dbtools.init_object_databases(couch)
    bags = dbs['bags']
    if args.compute_all:
        models.sync_models(dbs)
        results = models.Bag.all(bags)
        for bag in results:
            existing_sessions = models.Session.by_bag_id(dbs['sessions'], key=bag.id)
            if(len(existing_sessions) == 0):
                obj = models.Object.load(dbs['objects'], bag.object_id)
                print "Computing session for:", obj.object_name, "\ndescription:", obj.description
                compute_for_bag(bag, bags, args)
            else:
                print "Skipping bag:", bag.id, "Already computed %d sessions" % len(existing_sessions)
    else:
        bag = models.Bag.load(bags, args.bag_id)
        if bag == None or bag.id == None:
            print "Could not load bag with id:", args.bag_id
            sys.exit(-1)
        session = compute_for_bag(bag, bags, args)
        print "Calculated session_id =", session.id


