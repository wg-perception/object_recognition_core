#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc
from fiducial_pose_est import *
#lil bit o ros
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs
import sys
import argparse

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

if "__main__" == __name__:
    ecto_ros.init(sys.argv, "data_capture")
  
    plasm = ecto.Plasm()
    sched = ecto.schedulers.Singlethreaded(plasm)
    
    debug = True
    poser = OpposingDotPoseEstimator(plasm,
                                        rows=5, cols=3,
                                        pattern_type="acircles",
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
    print gray2rgb.__doc__
    

    display = highgui.imshow('Poses', name='Poses', waitKey=2, autoSize=True)
    mask_display = highgui.imshow('Masks', name='Masks', waitKey= -1, autoSize=True)
    object_display = highgui.imshow('Object', name='Object', waitKey= -1, autoSize=True)

    imsaver = highgui.ImageSaver('pose image saver', filename='image_pose_')
    rawsaver = highgui.ImageSaver('raw image saver', filename='image_raw_')
    masker = calib.PlanarSegmentation(z_min=0.015, y_crop=0.15, x_crop=0.15)
    bitwise_and = imgproc.BitwiseAnd()
    
    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()
    
    db_inserter = objcog_db.ObservationInserter("db_inserter", object_id="object_01")
                      
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
                  bitwise_and[:] >> object_display[:]
                  )
                
    ecto.view_plasm(plasm)
    sched.execute()

