#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc
import capture
import time
import reconstruction
import sys
import subprocess
import object_recognition_db
import argparse
import os
import math

def surfel_reconstruction(object_id, session_id, viz=True):
    db_reader = capture.ObservationReader('db_reader', session_id=session_id)
    depthTo3d = calib.DepthTo3d()
    surfel_reconstruction = reconstruction.SurfelReconstruction(corrDistForUpdate=0.005,
                                                                starvationConfidence=4,
                                                                timeDiffForRemoval=10,
                                                                maxNormalAngle=90 * math.pi / 180)
    if True:
        plasm = ecto.Plasm()
        plasm.connect(
                      db_reader['K', 'depth'] >> depthTo3d['K', 'depth'],
                      depthTo3d['points3d'] >> surfel_reconstruction['points3d'],
                      db_reader['K', 'R', 'T', 'image', 'mask'] >> surfel_reconstruction['K', 'R', 'T', 'image', 'mask'],
                      )
        if viz:
            plasm.connect(
                      db_reader['image'] >> highgui.imshow('image', name='image', waitKey=10, autoSize=True)[:],
                      db_reader['depth'] >> highgui.imshow('depth', name='depth', waitKey= -1, autoSize=True)[:],
                      db_reader['mask'] >> highgui.imshow('mask', name='mask', waitKey= -1, autoSize=True)[:],
                      )
        sched = ecto.schedulers.Singlethreaded(plasm)
        sched.execute()

    if True:
        location = '/tmp/object_recognition/'
        mesh_script_txt = '''<!DOCTYPE FilterScript>
<FilterScript>
 <filter name="Surface Reconstruction: Poisson">
  <Param type="RichInt" value="8" name="OctDepth"/>
  <Param type="RichInt" value="8" name="SolverDivide"/>
  <Param type="RichFloat" value="2" name="SamplesPerNode"/>
  <Param type="RichFloat" value="1" name="Offset"/>
 </filter>
</FilterScript>     
        '''
        mesh_file_name = os.path.join(location, 'meshing.mlx')
        with open(mesh_file_name, 'w') as mesh_script:
            mesh_script.write(mesh_script_txt)
        try:
            os.makedirs(location)
        except Exception, e:
            pass
        surfel_ply = os.path.join(location, object_id + '_' + session_id + '.ply')
        meshed_ply = os.path.join(location, object_id + '_' + session_id + '_meshed.ply')
        plasm = ecto.Plasm()
        surfel_saver = reconstruction.SurfelToPly(filename=surfel_ply)
        surfel_saver.inputs.at('model').copy_value(surfel_reconstruction.outputs.at('model'))
        surfel_saver.inputs.at('params').copy_value(surfel_reconstruction.outputs.at('params'))
        surfel_saver.inputs.at('camera_params').copy_value(surfel_reconstruction.outputs.at('camera_params'))

        plasm.insert(surfel_saver)
        sched = ecto.schedulers.Singlethreaded(plasm)
        sched.execute(niter=1)

        mesh_args = ["meshlabserver", "-i", surfel_ply, "-o", meshed_ply, "-s", mesh_file_name, "-om", "vn", "fn", "vc", "fc"]
        p = subprocess.Popen(mesh_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        o, e = p.communicate()
        if p.returncode :
            raise (o, e, p.returncode)

        db_url = 'http://localhost:5984'
        reconstruction.insert_mesh(db_url, object_id, session_id, meshed_ply, surfel_ply)
def find_object_id(session_id):
    db_url = 'http://localhost:5984'

    find_object_id = '''
    function(doc)
    {
      if(doc.session_id == "%s")
          emit('object_id',doc.object_id);
    }
    '''
    objects = object_recognition_db.run_view(db_url, 'sessions', find_object_id % session_id)
    print objects
    if len(objects) > 0:
        return objects[0][2].split('"')[1]
    else:
        raise "No object found with the given session:", session_id


def find_sessions_that_need_meshing():
    db_url = 'http://localhost:5984'

    find_all_objects = '''
            function(doc)
            {
              emit('tags',doc.tags);
            }
    '''


    find_all_sessions = '''
            function(doc)
            {
              if(doc.object_id == "%s")
                  emit('session_id',doc.session_id);
            }
    '''
    find_all_meshes = '''
            function(doc)
            {
              if(doc.session_id == "%s")
                  emit('session_id',doc.session_id);
            }
    '''
    objects = object_recognition_db.run_view(db_url, 'objects', find_all_objects)

    sessions_to_do = []
    for object in objects:
        print object
        sessions = object_recognition_db.run_view(db_url, 'sessions', find_all_sessions % object[0])
        for session in sessions:
            print session
            meshes = object_recognition_db.run_view(db_url, 'meshes', find_all_meshes % session[0])
            if len(meshes) == 0: #we haven't yet computed a mesh.
                sessions_to_do.append((object[0], session[0]))
    return sessions_to_do

def parse_args():
    parser = argparse.ArgumentParser(description='Reconstruct 3D meshes of objects from the object recognition db.')
    parser.add_argument('-s', '--session_id', metavar='SESSION_ID', dest='session_id', type=str, default='',
                       help='The session id to reconstruct.')
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()

    if len(args.session_id) > 0:
        object_id = find_object_id(args.session_id)
        surfel_reconstruction(object_id=object_id, session_id=args.session_id)
    else:
        sessions_to_do = find_sessions_that_need_meshing()
        for object_id, session_id in sessions_to_do:
            print object_id, session_id
            surfel_reconstruction(object_id=object_id, session_id=session_id)

