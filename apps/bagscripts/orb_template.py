#!/usr/bin/env python
import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer
from ecto_opencv.features2d import ORB, DrawKeypoints, Matcher, MatchRefinement, MatchRefinement3d, DrawMatches

from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import LatchMat, Select3d
from object_recognition.common.io.source import Source, SourceTypes
from ecto_object_recognition.tod_training import KeypointsToMat
import argparse
from ecto.opts import scheduler_options, run_plasm
import ecto_ros
import sys


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


n_features = 1000

parser = argparse.ArgumentParser(description='Computes the ORB feature and descriptor on a video stream.')
scheduler_options(parser)
Source.add_arguments(parser)
options = parser.parse_args()
source = Source.parse_arguments(options)

draw_kpts = DrawKeypoints()
rgb2gray = cvtColor (flag=Conversion.RGB2GRAY)
latch = LatchMat()
fps = FPSDrawer()

orb_display = imshow('orb display', name='ORB', triggers=dict(save=ord('s')))
plasm = ecto.Plasm()

plasm.connect(source['image'] >> rgb2gray ['image'])
img_src = rgb2gray['image']
orb_test = FeatureFinder('ORB test', n_features=n_features)

plasm.connect(
              img_src >> orb_test['image'],
              source['depth'] >> imshow(name='depth')[:],
              source['points3d'] >> orb_test['points3d'],
              orb_test['keypoints'] >> draw_kpts['keypoints'],
              source['image'] >> draw_kpts['image'],
              draw_kpts['image'] >> fps[:],
              fps[:] >> orb_display['image'],
              )

orb_train = ecto.If("ORB training", cell=FeatureFinder(n_features=n_features * 2, scale_factor=1.1, n_levels=10).__impl)
plasm.connect(orb_display['save'] >> (latch['set'], orb_train['__test__']),
              source['points3d'] >> orb_train['points3d'],
              img_src >> latch['input'],
              latch['output'] >> imshow(name='latched')['image'],
              latch['output'] >> orb_train['image'],
              )
matcher = ecto.If('Matcher', cell=Matcher())
match_refinement = ecto.If('Match 3D ~> 3D', cell=MatchRefinement())
match_drawer = ecto.If('Match Drawing', cell=DrawMatches())

for x in (matcher, match_refinement, match_drawer):
    plasm.connect(latch['set'] >> x['__test__'])

plasm.connect(orb_test['descriptors'] >> matcher['test'],
              orb_train['descriptors'] >> matcher['train'],
              match_refinement['matches'] >> match_drawer['matches'],
              orb_test['keypoints'] >> match_drawer['test'],
              orb_train['keypoints'] >> match_drawer['train'],
              orb_test['keypoints'] >> match_refinement['test'],
              orb_train['keypoints'] >> match_refinement['train'],
#              orb_test['points3d'] >> match_refinement['test'],
#              orb_train['points3d'] >> match_refinement['train'],
              matcher['matches'] >> match_refinement['matches'],
              match_refinement['matches_mask'] >> match_drawer['matches_mask'],
              
              img_src >> match_drawer['test_image'],
              latch['output'] >> match_drawer['train_image'],
              match_drawer['output'] >> imshow(name='matches')['image']
              )



ecto_ros.init(sys.argv, 'data_capture')
run_plasm(options, plasm, locals=vars())
