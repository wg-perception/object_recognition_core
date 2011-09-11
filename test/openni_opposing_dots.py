#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc, cv_bp as cv
from object_recognition.observations import *
import ecto_ros, ecto_sensor_msgs
from ecto_object_recognition import capture

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

plasm = ecto.Plasm()

subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
            depth=ImageSub(topic_name='/camera/depth_registered/image', queue_size=0),
            image_ci=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
            depth_ci=CameraInfoSub(topic_name='/camera/depth_registered/camera_info', queue_size=0),
            )

sync = ecto_ros.Synchronizer('Synchronizator', subs=subs
                             )

im2mat_rgb = ecto_ros.Image2Mat('rgb -> cv::Mat')
camera_info = ecto_ros.CameraInfo2Cv('camera_info -> cv::Mat')
poser = OpposingDotPoseEstimator(plasm,
                                 rows=5, cols=3,
                                 pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                 square_size=0.04, debug=True)

bgr2rgb = imgproc.cvtColor('rgb -> bgr', flag=imgproc.Conversion.RGB2BGR)
rgb2gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.Conversion.RGB2GRAY)
display = highgui.imshow('Poses', name='Poses')

graph = [sync['image'] >> im2mat_rgb[:],
          im2mat_rgb[:] >> (rgb2gray[:], bgr2rgb[:]),
          bgr2rgb[:] >> poser['color_image'],
          rgb2gray[:] >> poser['image'],
          poser['debug_image'] >> display['input'],
          sync['image_ci'] >> camera_info['camera_info'],
          camera_info['K'] >> poser['K'],
          ]
plasm.connect(graph)

if __name__ == '__main__':
    import sys
    ecto_ros.init(sys.argv, "opposing_dots_pose", False)
    from ecto.opts import doit
    doit(plasm, description='Estimate the pose of an opposing dot fiducial.')

