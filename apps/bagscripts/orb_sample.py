#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer, MatPrinter
from ecto_opencv.features2d import ORB, DrawKeypoints, Matcher, MatchRefinement, MatchRefinement3d, DrawMatches

from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import LatchMat, Select3d, Select3dRegion, PlaneFitter, PoseDrawer
from object_recognition.common.io.source import Source, SourceTypes
from ecto_object_recognition.tod_training import KeypointsToMat
import argparse
from ecto.opts import scheduler_options, run_plasm
import ecto_ros
import sys


def parse_args():
    parser = argparse.ArgumentParser(description='Computes the ORB feature and descriptor on a video stream.')
    scheduler_options(parser)
    Source.add_arguments(parser)
    options = parser.parse_args()
    return options

class FeatureFinder(ecto.BlackBox):
    orb = ORB
    keypointsTo2d = KeypointsToMat
    select3d = Select3d

    def declare_params(self, p):
        p.forward_all('orb')

    def declare_io(self, p, i, o):
        i.forward('points3d', 'select3d')
        i.forward_all('orb')
        o.forward_all('orb')
        o.forward_all('keypointsTo2d')
        o.forward_all('select3d')

    def connections(self):
        return [self.orb['keypoints'] >> self.keypointsTo2d['keypoints'],
                self.keypointsTo2d['points'] >> self.select3d['points'],
                ]

class PlaneEstimator(ecto.BlackBox):
    #find a plane in the center region of the image.
    region = Select3dRegion
    plane_fitter = PlaneFitter
    flag = ecto.Passthrough
            
    def declare_params(self, p):
        p.forward_all('region')

    def declare_io(self, p, i, o):
        i.forward_all('region')
        i.forward('set','flag',cell_key='in')
        o.forward_all('plane_fitter')

    def connections(self):
        return [ self.region['points3d'] >> self.plane_fitter['points3d'],
                self.plane_fitter['R'] >> MatPrinter(name='R')[:],
                self.plane_fitter['T'] >> MatPrinter(name='T')[:],
                ]

n_features = 2000
n_features_train = 5000


options = parse_args()

plasm = ecto.Plasm()

#setup the input source, grayscale conversion
source = Source.parse_arguments(options)
rgb2gray = cvtColor (flag=Conversion.RGB2GRAY)

plasm.connect(source['image'] >> rgb2gray ['image'])

plane_est = PlaneEstimator(radius=50)
pose_draw = PoseDrawer()
plasm.connect(source['image', 'points3d'] >> plane_est['image', 'points3d'],
                plane_est['R','T'] >> pose_draw['R', 'T'],
                source['K', 'image'] >> pose_draw['K', 'image'],
                pose_draw['output'] >> imshow(name='plane')[:]
                )

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
              fps[:] >> orb_display['image'],
             )

#training 
orb_train = ecto.If("ORB training", cell=FeatureFinder(n_features=n_features_train, scale_factor=1.1, n_levels=10).__impl)
latch = LatchMat()
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
