#!/usr/bin/env python
"""
module that loads all the NIST bags and puts them in the DB
"""

from argparse import ArgumentParser
import os
from object_recognition_core.db.tools import add_db_arguments
import subprocess
from object_recognition.ingest.bag_upload import upload_bag
from object_recognition import models
import random
import ecto
import ecto_ros
import ecto_sensor_msgs
import sys

ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo
PointCloud2Bagger = ecto_sensor_msgs.Bagger_PointCloud2

NIST_IDS = {1: 'Odwalla Orange Juice', 2: 'Odwalla Summertime Lime', 3: 'Spam Original', 4: 'Learning OpenCV',
            5: 'Mop & Glow', 6: 'Silk Original soy milk', 7: 'Claritin-D 24 hour', 8: "Hershey's Coca Special Dark",
            9: 'Tazo Organic Chai tea', 10: 'Good Earth Original', 11: "Snedd's Spread Country Crock (margarine)",
            12: "Kellogg's Raisin Brand", 13: 'Tilex Mold & Mildew Remover', 14: 'Arm & Hammer Detergent',
            15: 'Tide detergent', 16: 'Downy detergent', 17: 'All detergent', 18: 'Nestle Coffee Mate French Vanilla',
            19: 'Gillette complete skin care shaving cream', 20: 'All 3x ultra detergent',
            21: 'General Mills Oatmeal Crisp', 22: 'Del Monte Peas & Carrots',
            23: "Campbell's soup at hand creamy tomato", 24: 'Clorox regular bleach', 25: 'Coke',
            26: 'Ziploc plastic bags 6 1/2 x 5 7/8in (16.5 x 14.9cm)', 27: "Campbell's condensed tomato soup",
            28: 'Progresso Traditional New England Clam Chowder', 29: "Campell's just heat & engjoy tomato soup",
            30: 'Crest tooth paste tarter control plus scope', 31: 'Colgate toothpaste Icy Blast',
            32: 'Bumble BEe Chunk White albacore tuna in water', 33: 'Band-Aid plastic strips Johnson&Johnson',
            34: 'Jell-o Strawberry', 35: 'Tropicana 100% pure Orange Juice with calcium (6 pack)'}

def convert_bag(folder, file_name):
    bag_path = '%s/%s' % (folder, file_name)
    # TOD use that one and delete
    tmp_bag_path = bag_path + str(random.random()) + '.tmp'
    tmp_bag_path = '/tmp/' + str(random.random()) + '.tmp'
    print 'converting the bags to the new format in file "%s"' % tmp_bag_path

    baggers_reader = dict(info=CameraInfoBagger(topic_name='camera_info'),
                          rgb=ImageBagger(topic_name='image_color'),
                          depth=PointCloud2Bagger(topic_name='points'),
                          )
    bagreader = ecto_ros.BagReader('bag reader',
                                baggers=baggers_reader,
                                bag=bag_path)

    baggers_writer = dict(info_rgb=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
                          info_depth=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
                          rgb=ImageBagger(topic_name='/camera/rgb/image_color'),
                          depth=ImageBagger(topic_name='/camera/depth/image')
                          )
    bagwriter = ecto_ros.BagWriter('bag writer',
                                baggers=baggers_writer,
                                bag=tmp_bag_path)

    pcd_msg2depth_msg = ecto_ros.PointCloud22DepthImage()
    plasm = ecto.Plasm()
    plasm.connect([bagreader['info'] >> bagwriter['info_rgb'],
                    bagreader['info'] >> bagwriter['info_depth'],
                    bagreader['rgb'] >> bagwriter['rgb'],
                    bagreader['depth'] >> pcd_msg2depth_msg['cloud'],
                    pcd_msg2depth_msg['image'] >> bagwriter['depth']
                    ])
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
    return tmp_bag_path

if __name__ == '__main__':
    ecto_ros.init(sys.argv, "ecto_node")

    parser = ArgumentParser()
    parser.add_argument('--folder', dest='folder', help='The Folder where all the NIST bags are located', required=True)
    add_db_arguments(parser)
    args = parser.parse_args()

    # process all the bags in the folder
    for file_name in os.listdir(args.folder):
        if not file_name.endswith('.bag'):
            continue
        # persist the bad to the DB
        object_index = int(file_name[7:9])

        print '------- %s' % file_name

        bag_path = convert_bag(args.folder, file_name)

        print 'uploading file "%s"' % file_name
        obj = models.Object(object_name=NIST_IDS[object_index],
                        description=NIST_IDS[object_index],
                        tags='nist',
                        author_name='NIST',
                        author_email='',
                        )
        bag_file = open(bag_path, 'rb')
        bag = upload_bag(obj, bag_file, args.db_root)

        # Compute the observations
        print 'computing the observations'
        command = [ '../../compute_observations.py', '-i', bag.id, '--db_root', args.db_root ]
        stdout, stderr = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0:2]
        print stdout
        print stderr
        if stderr:
            print stderr
        break
