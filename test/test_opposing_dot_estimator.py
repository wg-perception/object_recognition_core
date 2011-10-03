#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc, cv_bp as cv
from object_recognition.observations import *
from ecto_object_recognition import capture

plasm = ecto.Plasm()

video_cap = highgui.VideoCapture(video_device=0, width=640, height=480)
camera_intrinsics = calib.CameraIntrinsics(camera_file='camera.yml')

poser = OpposingDotPoseEstimator(rows=5, cols=3,
                                 pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                 square_size=0.04, debug=True
                                 )

bgr2rgb = imgproc.cvtColor('rgb -> bgr', flag=imgproc.RGB2BGR)
rgb2gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.RGB2GRAY)
display = highgui.imshow('Poses', name='Poses')

plasm.connect(
         video_cap['image'] >> rgb2gray[:], 
         rgb2gray[:] >> poser['image'],
         video_cap['image'] >> poser['color_image'],
         poser['debug_image'] >> display['image'],
         camera_intrinsics['K'] >> poser['K']
         )

if __name__ == '__main__':
    from ecto.opts import doit
    doit(plasm, description='Estimate the pose of an opposing dot fiducial.')

