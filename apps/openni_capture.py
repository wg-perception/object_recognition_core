#!/usr/bin/env python
# abstract the input.
import ecto
from object_recognition import capture
from object_recognition.capture import openni_capture

import ecto_ros
import sys
import argparse
import math
import textwrap

def parse_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                    description=textwrap.dedent(
'''Captures data appropriate for training object recognition pipelines.
Assumes an opposing dot pattern, (black dots on white background, white
dots on black background), is the scene, and captures views of the 
object sparsely, depending on the delta setting.'''),
                                     epilog=textwrap.dedent(
    '''Capturing requires a ROS openni device (http://ros.org/wiki/openni),
with registered rgb and depth. You may want to do the following before 
capturing data with this program:

    $ roslaunch openni_launch openni.launch

    In a seperate terminal:

    $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True

    To switch into high resolution mode:

    $ rosrun dynamic_reconfigure dynparam set /camera/driver image_mode 1

    To switch to vga mode:

    $ rosrun dynamic_reconfigure dynparam set /camera/driver image_mode 2
''')
                                     )

    parser.add_argument('-o', '--output', metavar='BAG_FILE', dest='bag', type=str,
                       default='',
                       help='A bagfile to write to.')
    parser.add_argument('-a', '--angle_thresh', metavar='RADIANS', dest='angle_thresh', type=float,
                       default=math.pi / 36,
                       help='''The delta angular threshold in pose. Default is pi/36 radians.
                            Frames will not be recorded unless they are not closer to any other pose by this amount.
                            ''')
    args = parser.parse_args()
    if len(args.bag) < 1:
      print parser.print_help()
      sys.exit(1)
    return args

if "__main__" == __name__:
    ecto_ros.init(sys.argv, "openni_capture", False)
    args = parse_args()
    plasm = openni_capture.create_capture_plasm(args.bag, args.angle_thresh)

    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()
