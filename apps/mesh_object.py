#!/usr/bin/env python
import roscompat
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
from object_recognition import dbtools, models, capture
from ecto_object_recognition import reconstruction
import ecto_pcl


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
    object_recognition.dbtools.add_db_options(parser)
    args = parser.parse_args()
    if args.compute_all == False and args.session_id == '':
        parser.print_help()
        sys.exit(1)
    return args


def simple_mesh_session(dbs, session, args):
    obs_ids = models.find_all_observations_for_session(dbs, session.id)
    if len(obs_ids) == 0:
        raise RuntimeError("There are no observations available.")
    db_reader = capture.ObservationReader('db_reader', db_url=args.db_root, db_collection=args.db_collection)
    #observation dealer will deal out each observation id.
    observation_dealer = ecto.Dealer(typer=db_reader.inputs.at('observation'), iterable=obs_ids)
    depthTo3d = calib.DepthTo3d()
    erode = imgproc.Erode(kernel=3) #-> 7x7
    rescale_depth = capture.RescaledRegisteredDepth() #this is for SXGA mode scale handling.
    point_cloud_transform = reconstruction.PointCloudTransform()
    plasm = ecto.Plasm()
    plasm.connect(
      observation_dealer[:] >> db_reader['observation'],
      db_reader['K'] >> depthTo3d['K'],
      db_reader['image'] >> rescale_depth['image'],
      db_reader['depth'] >> rescale_depth['depth'],
      rescale_depth[:] >> depthTo3d['depth'],
      depthTo3d['points3d'] >> point_cloud_transform['points3d'],
      db_reader['mask'] >> erode['image'],
      db_reader['R', 'T', 'image'] >> point_cloud_transform['R', 'T', 'image'],
      erode['image'] >> point_cloud_transform['mask'],
      )


    accum = reconstruction.PointCloudAccumulator()
    viewer = ecto_pcl.CloudViewer("viewer", window_name="PCD Viewer")
    voxel_grid = ecto_pcl.VoxelGrid("voxel_grid", leaf_size=0.0075)
    outlier_removal = ecto_pcl.StatisticalOutlierRemoval('Outlier Removal', mean_k=2, stddev=2)

    source, sink = ecto.EntangledPair(value=accum.inputs.at('view'), source_name='Feedback Cloud', sink_name='Feedback Cloud')
    ply_writer = ecto_pcl.PLYWriter()

    plasm.connect(source[:] >> accum['previous'],
                  point_cloud_transform['view'] >> accum['view'],
                          accum[:] >> voxel_grid[:],
                          voxel_grid[:] >> outlier_removal[:],
                         # outlier_removal[:] >> normals[:],
                          outlier_removal[:] >> (sink[:], viewer[:], ply_writer[:]),
    )

    if args.visualize:
        plasm.connect(
          db_reader['image'] >> highgui.imshow('image', name='image')[:],
          db_reader['depth'] >> highgui.imshow('depth', name='depth')[:],
          erode['image'] >> highgui.imshow('mask', name='mask')[:],
          )
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

if "__main__" == __name__:
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    dbs = dbtools.init_object_databases(couch)
    sessions = dbs
    if args.compute_all:
        models.sync_models(dbs)
        results = models.Session.all(sessions)
        results = models.Session.all(sessions)

        for session in results:
            simple_mesh_session(dbs, session, args)
    else:
        session = models.Session.load(sessions, args.session_id)
        if session == None or session.id == None:
            print "Could not load session with id:", args.session_id
            sys.exit(1)
        simple_mesh_session(session, args)
