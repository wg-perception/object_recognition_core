#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc
import capture
import time
import reconstruction
import sys
import subprocess
db_reader = capture.ObservationReader('db_reader', object_id=sys.argv[1])
depthTo3d = calib.DepthTo3d()
surfel_reconstruction = reconstruction.SurfelReconstruction(corrDistForUpdate=0.01,
                                                            starvationConfidence=4)
pass_through = ecto.Passthrough()

if True:
    plasm = ecto.Plasm()
    plasm.connect(
                  db_reader['K', 'depth'] >> depthTo3d['K', 'depth'],
                  depthTo3d['points3d'] >> surfel_reconstruction['points3d'],
                  db_reader['K', 'R', 'T', 'image', 'mask'] >> surfel_reconstruction['K', 'R', 'T', 'image', 'mask'],
                  )
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

if True:
    surfel_ply = sys.argv[1] + '.ply'
    meshed_ply = sys.argv[1] + '_meshed.ply'
    plasm = ecto.Plasm()
    surfel_saver = reconstruction.SurfelToPly(filename=surfel_ply)
    surfel_saver.inputs.at('model').copy_value(surfel_reconstruction.outputs.at('model'))
    surfel_saver.inputs.at('params').copy_value(surfel_reconstruction.outputs.at('params'))
    surfel_saver.inputs.at('camera_params').copy_value(surfel_reconstruction.outputs.at('camera_params'))

    plasm.insert(surfel_saver)
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute(niter=1)
    
    mesh_args = ["meshlabserver", "-i", surfel_ply, "-o", meshed_ply, "-s", "meshing.mlx", "-om", "vn", "fn", "vc", "fc"]
    p = subprocess.Popen(mesh_args,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    o, e = p.communicate()
    if p.returncode :
        raise o,e,p.returncode