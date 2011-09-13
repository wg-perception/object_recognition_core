#!/usr/bin/env python
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
                                     )

    parser.add_argument('-o', '--output', metavar='BAG_FILE', dest='bag', type=str,
                       default='',
                       help='A bagfile to write to.')
    parser.add_argument('-a', '--angle_thresh', metavar='RADIANS', dest='angle_thresh', type=float,
                       default=math.pi / 36,
                       help='''The delta angular threshold in pose. Default is pi/36 radians.
                            Frames will not be recorded unless they are not closer to any other pose by this amount.
                            ''')
    parser.add_argument('-c', '--camera_file', metavar='CAMERA_YML', dest='camera_file', type=str,
                       default='camera.yml',
                       help='''A yaml file that contains opencv matrices for camera_matrix, distortion_coefficients, and floats for image_width, image_height
                            ''')
    parser.add_argument('--preview', dest='preview', action='store_true',
                        default=False, help='Preview the pose estimator.')
    parser.add_argument('--use_turn_table', dest='use_turn_table', action='store_true',
                        default=False, help='Use an E106 servo based turntable.')
    from ecto.opts import scheduler_options
    #add ecto scheduler args.
    group = parser.add_argument_group('ecto scheduler options')
    scheduler_options(group, default_scheduler='Singlethreaded')
    args = parser.parse_args()
    if not args.preview and len(args.bag) < 1:
      print parser.print_help()
      sys.exit(1)
    return args

if "__main__" == __name__:
    argv = sys.argv[:]
    ecto_ros.strip_ros_args(sys.argv)
    options = parse_args()
    ecto_ros.init(argv, "openni_capture", False)

    plasm = None
    if options.preview:
        plasm = openni_capture.create_preview_capture_standalone(options.camera_file)
    else:
        (plasm, segmentation) = openni_capture.create_capture_plasm(bag_name=options.bag,
                                                                    angle_thresh=options.angle_thresh,
                                                                    z_min=0.02, y_crop=0.10, x_crop=0.10,
                                                                    n_desired=72,
                                                                    preview=options.preview,
                                                                    use_turn_table=options.use_turn_table)
    from ecto.opts import run_plasm
    run_plasm(options, plasm, locals=dict(plasm=plasm, segmentation=segmentation))
