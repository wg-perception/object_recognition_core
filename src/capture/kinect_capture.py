#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc, cv_bp as cv
import capture
from object_recognition.observations.fiducial_pose_est import *
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs
import sys
import argparse
import time
import math
from IPython.Shell import IPShellEmbed

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo


def parse_args():
    parser = argparse.ArgumentParser(description='Captures data appropriate for training object recognition pipelines.')
    parser.add_argument('-b', '--bag', metavar='BAG_FILE', dest='bag', type=str,
                       default='',
                       help='A bagfile to write to.')
    parser.add_argument('-d', '--delta', metavar='RADIANS', dest='delta', type=float,
                       default=math.pi/ 36,
                       help='''The delta angular threshold in pose. Default is pi/36 radians.  
Frames will not be recorded unless they are not closer to any other pose by this amount.
''')
    args = parser.parse_args()
    return args

if "__main__" == __name__:
    ecto_ros.init(sys.argv, "kinect_capture", False)
    args = parse_args()
    plasm = ecto.Plasm()
    sched = ecto.schedulers.Threadpool(plasm)

    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   image_ci=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
                   depth_ci=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
                   )
    
    bagwriter = ecto.If('Bag Writer if R|T',
                        cell= ecto_ros.BagWriter(baggers=baggers,bag=args.bag)
                        )
    
    subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
                depth=ImageSub(topic_name='/camera/depth/image', queue_size=0),
                image_ci=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
                depth_ci=CameraInfoSub(topic_name='/camera/depth/camera_info', queue_size=0),
                )
    
    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs
                                 )
    keys = subs.keys()
    graph = [
                sync[:] >> bagwriter[keys],
            ]
    plasm = ecto.Plasm()
    im2mat_rgb = ecto_ros.Image2Mat('rgb -> cv::Mat')
    camera_info = ecto_ros.CameraInfo2Cv('camera_info -> cv::Mat')
    poser = OpposingDotPoseEstimator(plasm,
                                     rows=5, cols=3,
                                     pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                     square_size=0.04, debug=True)
    rgb2gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.CV_RGB2GRAY)
    delta_pose = capture.DeltaRT("delta R|T", angle_thresh=args.delta)
    display = highgui.imshow('Poses', name='Poses', waitKey=5, autoSize=True)
    graph += [sync['image'] >> im2mat_rgb[:],
              im2mat_rgb[:] >> (rgb2gray[:], poser['color_image']),
              rgb2gray[:] >> poser['image'],
              poser['debug_image'] >> display['input'],
              sync['image_ci'] >> camera_info['camera_info'],
              camera_info['K'] >> poser['K'],
              poser['R', 'T', 'found'] >> delta_pose['R', 'T', 'found'],
              delta_pose['novel'] >> bagwriter['__test__'],
              ]
    plasm.connect(graph)
    print >>open("capture.dot","w"), plasm.viz()
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
