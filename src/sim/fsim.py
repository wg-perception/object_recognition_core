#!/usr/bin/env python

import ecto
from object_recognition.common.io.standalone.source import KinectReader
from object_recognition.observations import *
from ecto_opencv import highgui, calib, imgproc, cv_bp as cv
from object_recognition.observations import *
from ecto_object_recognition import capture, sim
import sys
board = 'board.png'
if len(sys.argv) > 1:
    board = sys.argv[1]

plasm = ecto.Plasm()

simulator = sim.PlanarSim(image_name=board, width=0.8, height=0.4)

poser = OpposingDotPoseEstimator(plasm,
                                 rows=5, cols=3,
                                 pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                 square_size=0.04, debug=True)

gt_pose_drawer = calib.PoseDrawer('Ground Truth Pose')

rgb2gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.Conversion.RGB2GRAY)
display = highgui.imshow(name='Pose')
depth_display = highgui.imshow(name='Depth')
graph = [simulator['image'] >> (rgb2gray[:], poser['color_image'], gt_pose_drawer['image']),
         simulator['R', 'T', 'K'] >> gt_pose_drawer['R', 'T', 'K'],
         gt_pose_drawer['output'] >> highgui.imshow(name="Ground Truth")['image'],
         rgb2gray[:] >> poser['image'],
         poser['debug_image'] >> display['image'],
         simulator['K'] >> poser['K'],
         simulator['depth'] >> depth_display[:],
         ]
plasm.connect(graph)

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()

#if __name__ == '__main__':
#    from ecto.opts import doit
#    doit(plasm, description='Simulate pose estimation')
