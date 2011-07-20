#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc
import capture
from fiducial_pose_est import *
#lil bit of ros
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs
import sys
import argparse
import time
from IPython.Shell import IPShellEmbed

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

if "__main__" == __name__:
    ecto_ros.init(sys.argv, "data_capture",False)
    parser = argparse.ArgumentParser(description='Captures data appropriate for training object recognition pipelines.')
    parser.add_argument('-i','--object_id', metavar='OBJECT_ID',  dest='object_id', type=str, default='object_%d'%int(time.time()),
                       help='The object id to insert into the db.')
    parser.add_argument('-d','--description', metavar='OBJECT_DESCRIPTION',dest='description',type=str,
                       default='An object',
                       help='Give the object a description. Quote please.')
    parser.add_argument('tags', metavar='TAG',nargs='+',type=str,
                       help='Tags to mark the object with.')
    args = parser.parse_args()
    plasm = ecto.Plasm()
    sched = ecto.schedulers.Singlethreaded(plasm)
    
    debug = True
    poser = OpposingDotPoseEstimator(plasm,
                                        rows=5, cols=3,
                                        pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                        square_size=0.04, debug=debug)
    camera_info = calib.CameraIntrinsics('Camera Info',
                                                  camera_file="camera.yml")
    subs = dict(image=ImageSub(topic_name='camera/rgb/image_color', queue_size=0),
                depth=ImageSub(topic_name='camera/depth/image', queue_size=0),
                )
    
    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs
                                 )

    brg2rgb = imgproc.cvtColor('bgr -> rgb', flag=4)
    rgb2gray = imgproc.cvtColor('rgb -> gray', flag=7)
    gray2rgb = imgproc.cvtColor('gray -> rgb', flag=8)

    display = highgui.imshow('Poses', name='Poses', waitKey=10, autoSize=True)
    mask_display = highgui.imshow('Masks', name='Masks', waitKey= -1, autoSize=True)
    object_display = highgui.imshow('Object', name='Object', waitKey= -1, autoSize=True)

    imsaver = highgui.ImageSaver('pose image saver', filename='image_pose_')
    rawsaver = highgui.ImageSaver('raw image saver', filename='image_raw_')
    masker = calib.PlanarSegmentation(z_min=0.015, y_crop=0.15, x_crop=0.15)
    bitwise_and = imgproc.BitwiseAnd()
    
    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()
    
    session_id = 'session_%d'%int(time.time())
    capture_description = "data_capture.py, given a fiducial produces views that are registered to the object with R|T and produces a binary mask."
    capture_tags = ['calibration','mask','intrinsics','extrinsics',
                    'depth','rgb']
    capture.insert_object(args.object_id,args.description, args.tags)
    capture.insert_session(session_id,args.object_id,capture_description, capture_tags)
    db_inserter = capture.ObservationInserter("db_inserter", object_id=args.object_id, session_id=session_id)
    plasm.connect(
                  sync["image"] >> im2mat_rgb["image"],
                  im2mat_rgb["image"] >> (brg2rgb[:],),
                  sync["depth"] >> im2mat_depth['image'],
                  brg2rgb[:] >> (rgb2gray[:], poser['color_image'], rawsaver['image'], bitwise_and['b']),
                  rgb2gray[:] >> poser['image'],
                  poser['debug_image'] >> (display['input'], imsaver['image']),
                  display['out'] >> (imsaver['trigger'], rawsaver['trigger']),
                  camera_info['K'] >> (poser['K'], masker['K']),
                  poser['R', 'T'] >> masker['R', 'T'],
                  im2mat_depth['image'] >> masker['depth'],
                  masker['mask'] >> (mask_display[:], gray2rgb[:]),
                  gray2rgb[:] >> bitwise_and['a'],
                  bitwise_and[:] >> object_display[:],
                  brg2rgb[:] >> db_inserter['image'],
                  im2mat_depth['image'] >> db_inserter['depth'],
                  bitwise_and[:] >> db_inserter['mask'],
                  poser['R', 'T','found'] >> db_inserter['R','T','found'],
                  camera_info['K'] >> db_inserter['K'],
                  )
                
    sched.execute()
