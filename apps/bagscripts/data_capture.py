#!/usr/bin/env python
import argparse
from object_recognition.capture import create_openni_bag_capture_plasm
from ecto.opts import scheduler_options, run_plasm
import ecto_ros
import sys
parser = argparse.ArgumentParser(description='Capture a bag of data from the kinect.')
scheduler_options(parser)
parser.add_argument('bagname', nargs=1, type=str)
options = parser.parse_args()
plasm = create_openni_bag_capture_plasm(options.bagname[0])
ecto_ros.init(sys.argv, 'data_capture')
run_plasm(options, plasm, locals)

