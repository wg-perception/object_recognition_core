#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc
import tod_db
from fiducial_pose_est import *
#lil bit of ros
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs
import sys
import argparse
import time

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

if "__main__" == __name__:
    ecto_ros.init(sys.argv, "data_capture",False)
    parser = argparse.ArgumentParser(description='Captures data appropriate for training object recognition pipelines.')
    parser.add_argument('--OBJECT_ID', metavar='OBJECT_ID',  dest='object_id', type=str, default='object_%d'%int(time.time()),
                       help='The object id to insert into the db.')
#    parser.add_argument('--frame_id', dest='frame_id', type=str, default='base',
#                       help='The frame id to associate this camera with.')
#    parser.add_argument('--threads', dest='threads', type=int, default=1,
#                       help='The number of threads to run with.')
    args = parser.parse_args()
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
    
    db_inserter = tod_db.ObservationInserter("db_inserter", object_id=args.object_id)
                      
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
                  poser['R', 'T'] >> db_inserter['R','T'],
                  camera_info['K'] >> db_inserter['K'],
                  )
                
    ecto.view_plasm(plasm)
    sched.execute()

'''
#!/bin/python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc
import tod
import tod_db
import time
#import orb as imgproc

debug = True

class PoseEstimator:
    
    def __init__(self):
        self.draw_debug = debug
        self.circle_drawer = calib.PatternDrawer("Found Pattern Drawer", rows=7, cols=3)
        self.pattern_show = highgui.imshow("Pattern view", name="pattern", waitKey= -1, autoSize=True)
        self.pose_drawer = calib.PoseDrawer("pattern pose")            
        self.rgb2gray = imgproc.cvtColor("rgb -> gray", flag=7)
        self.circle_detector = calib.PatternDetector("asymmetric dot detector", rows=7, cols=3, pattern_type="acircles", square_size=0.03)
        self.poser = calib.FiducialPoseFinder("Pose Estimation")
        self.camera_intrinsics = calib.CameraIntrinsics("camera.yml", camera_file="camera.kinect.vga.yml")
        
    def declare_graph(self, plasm, image_source):
        plasm.connect(image_source, "image", self.rgb2gray, "input")
        plasm.connect(self.rgb2gray, "out", self.circle_detector, "input")
        plasm.connect(self.camera_intrinsics, "K", self.poser, "K")
        plasm.connect(self.circle_detector, "out", self.poser, "points")
        plasm.connect(self.circle_detector, "ideal", self.poser, "ideal")
        plasm.connect(self.circle_detector, "found", self.poser, "found")
        if self.draw_debug:
            plasm.connect(image_source, "image", self.circle_drawer, "input")
            plasm.connect(self.circle_detector, "out", self.circle_drawer, "points")
            plasm.connect(self.circle_detector, "found", self.circle_drawer, "found")
        plasm.connect(self.poser, "R", self.pose_drawer, "R")
        plasm.connect(self.poser, "T", self.pose_drawer, "T")
        plasm.connect(self.circle_drawer, "out", self.pose_drawer, "image")
        plasm.connect(self.camera_intrinsics, "K", self.pose_drawer, "K")
        plasm.connect(self.pose_drawer, "output", self.pattern_show, "input")
        
plasm = ecto.Plasm()

pose_est = PoseEstimator()

capture = highgui.OpenNICapture(video_mode=opencv.CV_CAP_OPENNI_VGA_30HZ)
image_view = highgui.imshow(name="RGB", waitKey=10, autoSize=True)
mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
object_view = highgui.imshow("object_view", name="object_view", waitKey= -1, autoSize=True)

masker = tod.PlanarSegmentation(z_min=0)
if debug:
    depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
    plasm.connect(capture, "image", image_view , "input")
    plasm.connect(capture, "depth", depth_view , "input")
    
pose_est.declare_graph(plasm, capture)

plasm.connect(pose_est.poser, "R", masker, "R")
plasm.connect(pose_est.poser, "T", masker, "T")
plasm.connect(pose_est.camera_intrinsics, "K", masker, "K")
plasm.connect(capture, "depth", masker , "depth")
bitwise_and = imgproc.BitwiseAnd()
bitwise_and2 = imgproc.BitwiseAnd("image_ander")

#plasm.connect(masker,"mask",bitwise_and,"a")
#plasm.connect(capture,"valid",bitwise_and,"b")
#plasm.connect(bitwise_and,"out", mask_view,"input")

#plasm.connect(bitwise_and,"out",bitwise_and2,"a")
plasm.connect(masker, "mask", bitwise_and2, "a")

plasm.connect(pose_est.rgb2gray, "out", bitwise_and2, "b")
plasm.connect(bitwise_and2, "out", object_view, "input")

db_inserter = objcog_db.ObservationInserter("db_inserter", object_id="object_01")
plasm.connect(pose_est.poser, "R", db_inserter, "R")
plasm.connect(pose_est.poser, "T", db_inserter, "T")
plasm.connect(pose_est.camera_intrinsics, "K", db_inserter, "K")
plasm.connect(image_view, "out", db_inserter, "trigger")
plasm.connect(capture, "image", db_inserter , "image")
plasm.connect(capture, "depth", db_inserter , "depth")
plasm.connect(masker, "mask", db_inserter , "mask")

if debug:
  print plasm.viz()
  ecto.view_plasm(plasm)

    
while(image_view.outputs.out not in (27, ord('q'))):
    plasm.execute(1)
'''
