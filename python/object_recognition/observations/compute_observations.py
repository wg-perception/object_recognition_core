#!/usr/bin/env python
import roscompat
import sys
import argparse
import tempfile
import os

import couchdb

import ecto
from ecto_opencv import highgui, imgproc
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs

import object_recognition
from object_recognition import dbtools, models, capture

ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo
PoseBagger = ecto_geometry_msgs.Bagger_PoseStamped

def connect_observation_calc_with_mask_pose(sync, commit, object_id, session_id, debug=False):
    plasm = ecto.Plasm()
    #need to go from ROS message types to opencv types.
    depth_ci = ecto_ros.CameraInfo2Cv('camera_info -> cv::Mat')
    image_ci = ecto_ros.CameraInfo2Cv('camera_info -> cv::Mat')
    image = ecto_ros.Image2Mat()
    depth = ecto_ros.Image2Mat()
    mask = ecto_ros.Image2Mat()
    pose = ecto_ros.PoseStamped2RT()

    #conversions
    plasm.connect(
      sync["image"] >> image["image"],
      sync["depth"] >> depth['image'],
      sync['image_ci'] >> image_ci[:],
      sync['depth_ci'] >> depth_ci[:],
      sync['mask'] >> mask[:],
      sync['pose'] >> pose[:]
      )

    rgb = imgproc.cvtColor('bgr -> rgb', flag=imgproc.Conversion.BGR2RGB)
    gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.Conversion.RGB2GRAY)
    plasm.connect(image[:] >> (rgb[:], gray[:]),
                  )
    if debug:
        image_display = highgui.imshow('image display', name='image')
        mask_display = highgui.imshow('mask display', name='mask')
        plasm.connect(rgb[:] >> image_display[:])
        plasm.connect(mask[:] >> mask_display[:])

    if commit:
        db_inserter = capture.ObservationInserter("db_inserter", object_id=object_id, session_id=session_id)
        plasm.connect(depth[:] >> db_inserter['depth'],
                  pose['R', 'T'] >> db_inserter['R', 'T'],
                  mask[:] >> db_inserter['mask'],
                  rgb[:] >> db_inserter['image'],
                  image_ci['K'] >> db_inserter['K'],
                  )
    return plasm

def compute_for_bag_with_mask_pose(bag, bags, args):
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
                     mask=ImageBagger(topic_name='/camera/mask'),
                     pose=PoseBagger(topic_name='/camera/pose'),
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
        plasm = connect_observation_calc_with_mask_pose(sync, args.commit,
                                         str(session.object_id),
                                         str(session.id),
                                         args.visualize)
        sched = ecto.schedulers.Threadpool(plasm)
        sched.execute()
        return session
    finally:
        print "Removing tmp_file:", tmp_file.name
        os.remove(tmp_file.name)

def parse_args():
    parser = argparse.ArgumentParser(description='Computes observations from a raw bag of appropriate data.' +
                                     '  This assumes that the bag is already in the database associated' +
                                     ' with an object.',
                                      fromfile_prefix_chars='@')
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
                compute_for_bag_with_mask_pose(bag, bags, args) #TODO fixme , should we always assume that the bag has the mask and the pose?
            else:
                print "Skipping bag:", bag.id, "Already computed %d sessions" % len(existing_sessions)
    else:
        bag = models.Bag.load(bags, args.bag_id)
        if bag == None or bag.id == None:
            print "Could not load bag with id:", args.bag_id
            sys.exit(-1)
        session = compute_for_bag_with_mask_pose(bag, bags, args) #TODO fixme , should we always assume that the bag has the mask and the pose?
        print "Calculated session_id =", session.id


