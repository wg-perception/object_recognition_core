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
              source['mask'] >> orb_test['mask']
              )

#display test ORB
draw_kpts = DrawKeypoints()
fps = FPSDrawer()
orb_display = imshow('orb display', name='ORB', triggers=dict(save=ord('s')))
plasm.connect(orb_test['keypoints'] >> draw_kpts['keypoints'],
              source['image'] >> draw_kpts['image'],
              draw_kpts['image'] >> fps[:],
              fps[:] >> orb_display['image'],
             )

#training 
orb_train = ecto.If("ORB training", cell=FeatureFinder(n_features=n_features_train, scale_factor=1.1, n_levels=10).__impl)
latch = LatchMat()
R_latch = LatchMat()
T_latch = LatchMat()

orb_train = ecto.If("ORB training", cell=FeatureFinder(n_features=n_features_train, scale_factor=1.1, n_levels=10).__impl)
latch = LatchMat()
R_writer = ecto.If("R writer", cell=MatWriter(filename='R.yaml'))
T_writer = ecto.If("T writer", cell=MatWriter(filename='T.yaml'))

plasm.connect(orb_display['save'] >> (latch['set'], orb_train['__test__']),
              source['points3d'] >> orb_train['points3d'],
              img_src >> latch['input'],
              latch['output'] >> orb_train['image'],
              )
plasm.connect(orb_display['save'] >> (latch['set'], orb_train['__test__']),
              source['points3d'] >> orb_train['points3d'],
              img_src >> latch['input'],
              latch['output'] >> orb_train['image'],
              )

#display the latched image:
plasm.connect(latch['output'] >> imshow(name='latched')['image'])

#matching, dependent on the latch
matcher = ecto.If('Matcher', cell=Matcher())
match_refinement2d = ecto.If('Match H', cell=MatchRefinement())
match_refinement3d = ecto.If('Match 3D ~> 3D', cell=MatchRefinement3d())
match_drawer2d = ecto.If('Match Drawing H', cell=DrawMatches())
match_drawer3d = ecto.If('Match Drawing 3D', cell=DrawMatches())

for x in (matcher, match_refinement2d, match_refinement3d, match_drawer2d, match_drawer3d):
    plasm.connect(latch['set'] >> x['__test__'])

plasm.connect(orb_test['descriptors'] >> matcher['test'],
              orb_train['descriptors'] >> matcher['train'],
              )

#2d match refinement, fits a homography
plasm.connect(
              orb_test['keypoints'] >> match_refinement2d['test'],
              orb_train['keypoints'] >> match_refinement2d['train'],
              matcher['matches'] >> match_refinement2d['matches'],
              )

#3d estimation
plasm.connect(orb_test['points3d'] >> match_refinement3d['test'],
              orb_train['points3d'] >> match_refinement3d['train'],
              match_refinement2d['matches'] >> match_refinement3d['matches'],
              )

#display both Homography matches and 3D ~> 3D matches
for matches, match_drawer, display in ((match_refinement2d, match_drawer2d, imshow(name='H matches')),
                                       (match_refinement3d, match_drawer3d, imshow(name='3D matches'))
                                       ):
    #display matches
    plasm.connect(matches['matches'] >> match_drawer['matches'],
                  matches['matches_mask'] >> match_drawer['matches_mask'],
                  orb_test['keypoints'] >> match_drawer['test'],
                  orb_train['keypoints'] >> match_drawer['train'],
                  img_src >> match_drawer['test_image'],
                  latch['output'] >> match_drawer['train_image'],
                  match_drawer['output'] >> display['image']
              )

ecto_ros.init(sys.argv, 'data_capture')
run_plasm(options, plasm, locals=vars())
