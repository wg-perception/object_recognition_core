#!/usr/bin/env python
import ecto
import ecto_ros

from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer, MatPrinter, MatWriter
from ecto_opencv.features2d import ORB, DrawKeypoints, Matcher, MatchRefinement, MatchRefinementHSvd, MatchRefinement3d, MatchRefinementPnP, DrawMatches
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import LatchMat, Select3d, Select3dRegion, PlaneFitter, PoseDrawer, DepthValidDraw, TransformCompose
from ecto_object_recognition.tod_detection import LSHMatcher
from ecto.opts import scheduler_options, run_plasm


from object_recognition.common.io.source import Source, SourceTypes
from object_recognition.capture.orb_capture import OrbPoseEstimator

if __name__ == '__main__':
    import sys
    def parse_args():
        import argparse
        parser = argparse.ArgumentParser(description='Estimate the pose of an ORB template.')
        parser.add_argument('-i,--input', dest='input', type=str, help='The directory of the template to use. Default: %(default)s', default='./')
        parser.add_argument('-m,--matches', dest='matches', action='store_const',
                        const=True, default=False,
                        help='Visualize the matches.')

        scheduler_options(parser.add_argument_group('Scheduler'))
        Source.add_arguments(parser.add_argument_group('Source'))
        options = parser.parse_args()
        return options

    options = parse_args()
    plasm = ecto.Plasm()

    #setup the input source, grayscale conversion
    source = Source.parse_arguments(options)
    rgb2gray = cvtColor('Grayscale', flag=Conversion.RGB2GRAY)
    plasm.connect(source['image'] >> rgb2gray ['image'])

    pose_est = OrbPoseEstimator(directory=options.input, show_matches=options.matches)

    #convenience variable for the grayscale
    img_src = rgb2gray['image']

    #connect up the pose_est
    plasm.connect(img_src >> pose_est['image'],
                  source['image'] >> pose_est['color_image'],
                  source['points3d'] >> pose_est['points3d'],
                  source['mask'] >> pose_est['mask'],
                  source['K'] >> pose_est['K']
                  )

    display = imshow('orb display', name='Pose')
    plasm.connect(pose_est['debug_image'] >> display['image'],
                  )

    ecto_ros.init(sys.argv, 'data_capture',False)
    run_plasm(options, plasm, locals=vars())

