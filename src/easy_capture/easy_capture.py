#!/usr/bin/env python
'''
try:
roslaunch openni_launch openni.launch 
rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True

ensure that hz works on the depth_registered topic:
rostopic hz /camera/depth_registered/image

finally run:
python easy_capture.py -o test.bag --graphviz

'''
import ecto
from ecto_opencv import highgui, calib, imgproc, cv_bp as cv
import ecto_ros, ecto_sensor_msgs
import ecto_pcl, ecto_pcl_ros
from ecto_object_recognition import easy_capture

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
PointCloudSub = ecto_sensor_msgs.Subscriber_PointCloud2
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

def create_capture_plasm(bag_name):
    '''
    Creates a plasm that will capture openni data into a bag.
    '''
    # bag writing
    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   image_ci=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
                   depth_ci=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
                   )
    #conditional writer
    bagwriter = ecto.If('Bag Writer if \'s\'',
                        cell=ecto_ros.BagWriter(baggers=baggers, bag=bag_name)
                        )

    subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
                image_ci=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
                depth=ImageSub(topic_name='/camera/depth_registered/image', queue_size=0),
                cloud=PointCloudSub(topic_name='/camera/depth_registered/points', queue_size=0),
                depth_ci=CameraInfoSub(topic_name='/camera/depth_registered/camera_info', queue_size=0),
                )

    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs
                                 )

    graph = [
                sync['image','depth','image_ci','depth_ci'] >> bagwriter['image','depth','image_ci','depth_ci'],
            ]

    #point cloud stuff
    msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)
    example = easy_capture.ExampleFilter()
    graph += [
        sync['cloud'] >> msg2cloud[:],
        msg2cloud[:] >> example[:],
    ]

    #use opencv highgui to display an image.
    im2mat_rgb = ecto_ros.Image2Mat('rgb -> cv::Mat')
    display = highgui.imshow('Image Display', name='/camera/rgb/image_color',triggers=dict(save=ord('s')))
    bgr2rgb = imgproc.cvtColor('rgb -> bgr', flag=imgproc.Conversion.RGB2BGR)
    graph += [
        sync['image'] >> im2mat_rgb[:],
        im2mat_rgb[:] >> bgr2rgb[:],
        bgr2rgb[:] >> display[:],
        display['save'] >> bagwriter['__test__']
    ]

    plasm = ecto.Plasm()
    plasm.connect(graph)
    return plasm

def parse_args():
    ''' Setup some argparse stuffs.'''
    import argparse
    import textwrap
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                    description=textwrap.dedent('''
    Captures data appropriate for training object recognition pipelines.
    Press 's' to save a bundle of messages to the bag.'''),
                                     )

    parser.add_argument('-o', '--output', metavar='BAG_FILE', dest='bag', type=str,
                       default='',
                       help='A bagfile to write to.')

    from ecto.opts import scheduler_options
    #add ecto scheduler args.
    group = parser.add_argument_group('ecto scheduler options')
    scheduler_options(group, default_scheduler='Threadpool')
    args = parser.parse_args()
    if len(args.bag) < 1:
      print parser.print_help()
      print "Missing output filename."
      sys.exit(1)
    return args

if __name__ == '__main__':
    import sys
    argv = sys.argv[:]
    ecto_ros.strip_ros_args(sys.argv)
    options = parse_args()
    ecto_ros.init(argv, "openni_capture", False)
    plasm = create_capture_plasm(options.bag)
    from ecto.opts import run_plasm
    run_plasm(options, plasm)
