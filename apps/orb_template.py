#!/usr/bin/env python
import roscompat
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer, MatPrinter, MatWriter, ImageSaver
from ecto_opencv.features2d import ORB, DrawKeypoints, Matcher, MatchRefinement, MatchRefinement3d, DrawMatches

from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import LatchMat, Select3d, Select3dRegion, PlaneFitter, PoseDrawer, DepthValidDraw
from object_recognition.common.io.source import Source, SourceTypes
import argparse
from ecto.opts import scheduler_options, run_plasm
import ecto_ros
import sys
import os
from object_recognition.capture.orb_capture import *

def parse_args():
    parser = argparse.ArgumentParser(description='Computes the ORB feature and descriptor template that may be used as a fiducial marker.')
    parser.add_argument('-o,--output', dest='output', type=str, help='The output directory for this template. Default: %(default)s', default='./')
    parser.add_argument('-n_features', dest='n_features', type=int,
                        help='The number of features to detect for the template.,%(default)d',
                        default=5000)    
    scheduler_options(parser.add_argument_group('Scheduler'))
    Source.add_arguments(parser.add_argument_group('Source'))
    options = parser.parse_args()
    if not os.path.exists(options.output):
        os.makedirs(options.output)
    return options


options = parse_args()
plasm = ecto.Plasm()

#setup the input source, grayscale conversion
source = Source.parse_arguments(options)
rgb2gray = cvtColor (flag=Conversion.RGB2GRAY)

plasm.connect(source['image'] >> rgb2gray ['image'])

#convenience variable for the grayscale
img_src = rgb2gray['image']

#display the depth
plasm.connect(source['depth'] >> imshow(name='depth')[:],
              )

#connect up the test ORB
orb = FeatureFinder('ORB test', n_features=options.n_features, n_levels=3, scale_factor=1.2)
plasm.connect(img_src >> orb['image'],
              source['points3d'] >> orb['points3d'],
              source['mask'] >> orb['mask']
              )


#display test ORB
draw_kpts = DrawKeypoints()
fps = FPSDrawer()
orb_display = imshow('orb display', name='ORB', triggers=dict(save=ord('s')))
depth_valid_draw = DepthValidDraw()
plasm.connect(orb['keypoints'] >> draw_kpts['keypoints'],
              source['image'] >> depth_valid_draw['image'],
              source['mask'] >> depth_valid_draw['mask'],
              depth_valid_draw['image'] >> draw_kpts['image'],
              draw_kpts['image'] >> fps[:],
             )

plane_est = PlaneEstimator(radius=50)
pose_draw = PoseDrawer()
plasm.connect(source['image', 'points3d'] >> plane_est['image', 'points3d'],
              plane_est['R', 'T'] >> pose_draw['R', 'T'],
              source['K'] >> pose_draw['K'],
              fps[:] >> pose_draw['image'],
              pose_draw['output'] >> orb_display['image']
              )
#training 
points3d_writer = ecto.If("Points3d writer", cell=MatWriter(filename=os.path.join(options.output, 'points3d.yaml')))
points_writer = ecto.If("Points writer", cell=MatWriter(filename=os.path.join(options.output, 'points.yaml')))
descriptor_writer = ecto.If("Descriptor writer", cell=MatWriter(filename=os.path.join(options.output, 'descriptors.yaml')))
R_writer = ecto.If("R writer", cell=MatWriter(filename=os.path.join(options.output, 'R.yaml')))
T_writer = ecto.If("T writer", cell=MatWriter(filename=os.path.join(options.output, 'T.yaml')))
image_writer = ecto.If(cell=ImageSaver(filename=os.path.join(options.output, 'train.png')))

for y, x in (
            (orb['points3d'], points3d_writer),
            (orb['descriptors'], descriptor_writer),
            (orb['points'], points_writer),
            (plane_est['R'], R_writer),
            (plane_est['T'], T_writer)
            ):
    plasm.connect(orb_display['save'] >> x['__test__'],
                  y >> x['mat'],
              )
plasm.connect(orb_display['save'] >> image_writer['__test__'],
              source['image'] >> image_writer['image']
              )
if 'ros' in options.type:
  ecto_ros.init(sys.argv, 'orb_template')
run_plasm(options, plasm, locals=vars())
