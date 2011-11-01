#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer, MatPrinter, MatWriter
from ecto_opencv.features2d import ORB, DrawKeypoints, Matcher, MatchRefinement, MatchRefinement3d, DrawMatches

from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import LatchMat, Select3d, Select3dRegion, PlaneFitter, PoseDrawer
from object_recognition.common.io.source import Source, SourceTypes
from ecto_object_recognition.tod_training import KeypointsToMat
import argparse
from ecto.opts import scheduler_options, run_plasm
import ecto_ros
import sys
from orb_capture import *

def parse_args():
    parser = argparse.ArgumentParser(description='Computes the ORB feature and descriptor on a video stream.')
    scheduler_options(parser)
    Source.add_arguments(parser)
    options = parser.parse_args()
    return options

n_features = 2000
n_features_train = 5000


options = parse_args()

plasm = ecto.Plasm()

#setup the input source, grayscale conversion
source = Source.parse_arguments(options)
rgb2gray = cvtColor (flag=Conversion.RGB2GRAY)

plasm.connect(source['image'] >> rgb2gray ['image'])

#convenience variable for the grayscale
img_src = rgb2gray['image']

#display the depth
plasm.connect(
              source['depth'] >> imshow(name='depth')[:],
              )

#connect up the test ORB
orb_test = FeatureFinder('ORB test', n_features=n_features)
plasm.connect(img_src >> orb_test['image'],
              source['points3d'] >> orb_test['points3d'],
              )


#display test ORB
draw_kpts = DrawKeypoints()
fps = FPSDrawer()
orb_display = imshow('orb display', name='ORB', triggers=dict(save=ord('s')))

plasm.connect(orb_test['keypoints'] >> draw_kpts['keypoints'],
              source['image'] >> draw_kpts['image'],
              draw_kpts['image'] >> fps[:],
             )

plane_est = PlaneEstimator(radius=50)
pose_draw = PoseDrawer()
plasm.connect(source['image', 'points3d'] >> plane_est['image', 'points3d'],
                plane_est['R','T'] >> pose_draw['R', 'T'],
                source['K'] >> pose_draw['K'],
                fps[:] >> pose_draw['image'],
                pose_draw['output'] >> orb_display['image']
                )

#training 
points3d_writer = ecto.If("Feature writer", cell=MatWriter(filename='points3d.yaml.gz'))
descriptor_writer = ecto.If("Descriptor writer", cell=MatWriter(filename='descriptors.yaml.gz'))
R_writer = ecto.If("R writer", cell=MatWriter(filename='R.yaml'))
T_writer = ecto.If("T writer", cell=MatWriter(filename='T.yaml'))

for y,x in ((source['points3d'],points3d_writer),
            ( orb_test['descriptors'],descriptor_writer),
            (plane_est['R'],R_writer),
            (plane_est['T'],T_writer)
            ):
    plasm.connect(orb_display['save'] >> x['__test__'],
                  y >> x['mat'],
              )

ecto_ros.init(sys.argv, 'data_capture')
run_plasm(options, plasm, locals=vars())
