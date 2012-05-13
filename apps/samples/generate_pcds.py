#!/usr/bin/env python
import sys
import argparse
import time
import tempfile
import os
import math
import subprocess

import couchdb

import ecto
from ecto_opencv import calib, highgui, imgproc
import object_recognition
from object_recognition import tools as dbtools, models, capture
from ecto_object_recognition import reconstruction, conversion
import ecto_pcl
from tempfile import NamedTemporaryFile

cloud_view = ecto_pcl.CloudViewer("Cloud Display", window_name="PCD Viewer")[:]
imshows = dict(image=highgui.imshow('Image Display', name='image')[:], mask=highgui.imshow('Mask Display', name='mask')[:],
               depth=highgui.imshow('Depth Display', name='depth')[:])
def generate_pointclouds_in_object_space(dbs, session, args):
    object_name = dbs[session.object_id]['object_name']
    if not os.path.exists(object_name):
        os.mkdir(object_name)
    obs_ids = models.find_all_observations_for_session(dbs, session.id)
    if len(obs_ids) == 0:
        raise RuntimeError("There are no observations available.")
    db_reader = capture.ObservationReader('Database Source', db_params=dbtools.args_to_db_params(args))

    #observation dealer will deal out each observation id.
    observation_dealer = ecto.Dealer(tendril=db_reader.inputs.at('observation'), iterable=obs_ids)
    depthTo3d = calib.DepthTo3d('Depth ~> 3D')
    rescale_depth = capture.RescaledRegisteredDepth('Depth scaling') #this is for SXGA mode scale handling.
    point_cloud_transform = reconstruction.PointCloudTransform('Object Space Transform',do_transform=False)#keeps the points in camera coordinates, but populates the global sensor position and orientatino.
    point_cloud_converter = conversion.MatToPointCloudXYZRGB('To Point Cloud')
    to_ecto_pcl = ecto_pcl.PointCloudT2PointCloud('converter', format=ecto_pcl.XYZRGB)
    plasm = ecto.Plasm()
    plasm.connect(
      observation_dealer[:] >> db_reader['observation'],
      db_reader['K'] >> depthTo3d['K'],
      db_reader['image'] >> rescale_depth['image'],
      db_reader['depth'] >> rescale_depth['depth'],
      rescale_depth[:] >> depthTo3d['depth'],
      depthTo3d['points3d'] >> point_cloud_converter['points'],
      db_reader['image'] >> point_cloud_converter['image'],
      db_reader['mask'] >> point_cloud_converter['mask'],
      db_reader['R', 'T'] >> point_cloud_transform['R', 'T'],
      point_cloud_converter['point_cloud'] >> to_ecto_pcl[:],
      to_ecto_pcl[:] >> point_cloud_transform['cloud']
    )
    ply_writer = ecto_pcl.PLYWriter('PLY Saver',
                                      filename_format='%s/cloud_%%05d.ply' % (object_name))
    pcd_writer = ecto_pcl.PCDWriter('PCD Saver',
                                      filename_format='%s/cloud_%%05d.pcd' % (object_name))
    plasm.connect(point_cloud_transform['view'] >> (ply_writer['input'], pcd_writer['input'])
                  )

    if args.visualize:
        global cloud_view
        plasm.connect(
          point_cloud_transform['view'] >> cloud_view,
          db_reader['image'] >> imshows['image'],
          db_reader['depth'] >> imshows['depth'],
          db_reader['mask'] >> imshows['mask'],
          )

    from ecto.opts import run_plasm
    run_plasm(args, plasm, locals=vars())

###################################################################################################################
def parse_args():
    parser = argparse.ArgumentParser(description='Computes a surface mesh of an object in the database')
    parser.add_argument('-s', '--session_id', metavar='SESSION_ID', dest='session_id', type=str, default='',
                       help='The session id to reconstruct.')
    parser.add_argument('--all', dest='compute_all', action='store_const',
                        const=True, default=False,
                        help='Compute meshes for all possible sessions.')
    parser.add_argument('--visualize', dest='visualize', action='store_const',
                        const=True, default=False,
                        help='Turn on visiualization')
    object_recognition.dbtools.add_db_arguments(parser)

    sched_group = parser.add_argument_group('Scheduler Options')
    from ecto.opts import scheduler_options
    scheduler_options(sched_group, default_scheduler='Threadpool')

    args = parser.parse_args()
    if args.compute_all == False and args.session_id == '':
        parser.print_usage()
        print "You must either supply a session id, or --all."
        sys.exit(1)
    return args

if "__main__" == __name__:
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    dbs = dbtools.init_object_databases(couch)
    sessions = dbs
    if args.compute_all:
        models.sync_models(dbs)
        results = models.Session.all(sessions)
        for session in results:
            generate_pointclouds_in_object_space(dbs, session, args)
    else:
        session = models.Session.load(sessions, args.session_id)
        if session == None or session.id == None:
            print "Could not load session with id:", args.session_id
            sys.exit(1)
        generate_pointclouds_in_object_space(dbs, session, args)
