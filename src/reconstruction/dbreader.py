#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc
import capture
import time
import reconstruction
import sys

image_view = highgui.imshow(name='RGB', waitKey=5, autoSize=True)
db_reader = capture.ObservationReader('db_reader', object_id=sys.argv[1])
depthTo3d = calib.DepthTo3d()
surfel_reconstruction = reconstruction.SurfelReconstruction()
pass_through = ecto.Passthrough()

print 'computing'
if True:
    plasm = ecto.Plasm()
    plasm.connect(
                  db_reader['K', 'depth'] >> depthTo3d['K', 'depth'],
                  depthTo3d['points3d'] >> surfel_reconstruction['points3d'],
                  db_reader['K', 'R', 'T', 'image', 'mask'] >> surfel_reconstruction['K', 'R', 'T', 'image', 'mask'],
                  db_reader['image'] >> image_view[:],
                  )
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

print 'saving'
if True:
    plasm = ecto.Plasm()
    surfel_saver = reconstruction.SurfelToPly(filename=sys.argv[1]+'.ply')
    surfel_saver.inputs.at('model').copy_value(surfel_reconstruction.outputs.at('model'))
    surfel_saver.inputs.at('params').copy_value(surfel_reconstruction.outputs.at('params'))
    surfel_saver.inputs.at('camera_params').copy_value(surfel_reconstruction.outputs.at('camera_params'))

    plasm.insert(surfel_saver)
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(niter=1)
