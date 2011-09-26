#!/usr/bin/env python

import ecto
from ecto_opencv.highgui import imshow, VideoCapture
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import PatternDetector, FiducialPoseFinder, \
     PatternDrawer, PoseDrawer, CameraIntrinsics, ASYMMETRIC_CIRCLES_GRID
from sim import PlanarSim

import sys

board = '11x4.png'

rows = 11
cols = 4
width = 0.279
height = 0.216
offset_x = -(width/2 - 0.06)
offset_y = -(height/2 - 0.020)

print "ox" , offset_x, "oy", offset_y


simulator = PlanarSim(image_name=board, width=width, height=height)

square_size = 0.02 # in meters, 2 cm
pattern_type = ASYMMETRIC_CIRCLES_GRID

pattern_show = imshow('Display', name='pattern')
rgb2gray = cvtColor('RGB -> Gray', flag=Conversion.RGB2GRAY)
circle_detector = PatternDetector(rows=rows, cols=cols,
                                  pattern_type=pattern_type,
                                  square_size=square_size,
                                  offset_x = offset_x,
                                  offset_y = offset_y
                                  )

circle_drawer = PatternDrawer(rows=rows, cols=cols)
poser = FiducialPoseFinder()
pose_drawer = PoseDrawer()
gt_drawer = PoseDrawer()
plasm = ecto.Plasm()
plasm.connect(simulator['image'] >> (rgb2gray['image'], circle_drawer['input']),
            rgb2gray['image'] >> circle_detector['input'],
            circle_detector['out', 'found'] >> circle_drawer['points', 'found'],
            simulator['K'] >> poser['K'],
            circle_detector['out', 'ideal', 'found'] >> poser['points', 'ideal', 'found'],
            poser['R', 'T'] >> pose_drawer['R', 'T'],
            circle_detector['found'] >> pose_drawer['trigger'],
            circle_drawer['out'] >> pose_drawer['image'],
            simulator['K'] >> pose_drawer['K'],
            pose_drawer['output'] >> pattern_show['image'],
            )

plasm.connect(simulator['image','R','T','K'] >> gt_drawer['image','R','T','K'],
              gt_drawer['output'] >> imshow(name='Ground Truth')['image']
            )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Simulate pose estimation')
