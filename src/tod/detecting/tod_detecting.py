#!/usr/bin/env python
import ecto
import ecto_ros
import ecto_pcl
import ecto_pcl_ros
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, features2d
import ecto_sensor_msgs
import json
from optparse import OptionParser
import os
import time
import tod_db
import tod

DEBUG = False

class TodDetection(ecto.BlackBox):
    def __init__(self, plasm, feature_descriptor_json_params, db_json_params, object_ids, search_json_params,
                 guess_json_params):
        ecto.BlackBox.__init__(self, plasm)

        self._db_json_params = db_json_params
        self._feature_descriptor_json_params = feature_descriptor_json_params
        self._object_ids = object_ids
        self._guess_json_params = guess_json_params

        self.feature_descriptor = features2d.FeatureDescriptor(json_params=feature_descriptor_json_params)
        print object_ids
        self.descriptor_matcher = tod.DescriptorMatcher(db_json_params=db_json_params, object_ids=object_ids,
                                                        search_json_params=search_json_params)
        self.guess_generator = tod.GuessGenerator(json_params=guess_json_params)

    def expose_inputs(self):
        return {'image':self.feature_descriptor['image'],
                'mask':self.feature_descriptor['mask'],
                'point_cloud':self.guess_generator['point_cloud']}

    def expose_outputs(self):
        return {'object_ids': self.guess_generator['object_ids'],
                'poses': self.guess_generator['poses']}

    def expose_parameters(self):
        return {'db_json_params': self._db_json_params,
                'feature_descriptor_json_params': self._feature_descriptor_json_params,
                'guess_json_params': self._guess_json_params,
                'object_ids': self._object_ids
                }

    def connections(self):
        return (self.feature_descriptor['keypoints'] >> self.guess_generator['keypoints'],
                self.feature_descriptor['descriptors'] >> self.descriptor_matcher['descriptors'],
                self.descriptor_matcher['matches'] >> self.guess_generator['matches'],
                self.descriptor_matcher['matches_3d'] >> self.guess_generator['matches_3d'])

########################################################################################################################

def parse_options():
    parser = OptionParser()
    parser.add_option("-c", "--config_file", dest="config_file",
                      help="the file containing the configuration")
    parser.add_option("-a", "--bag", dest="bag", help="The bag to analyze")

    (options, args) = parser.parse_args()
    return options

########################################################################################################################


if __name__ == '__main__':
    options = parse_options()

    # define the input
    if options.config_file is None or not os.path.exists(options.config_file):
        raise 'option file does not exist'
    
    db_json_params = json.loads(str(open(options.config_file).read()))

    # define the input
    baggers = dict(image=ecto_sensor_msgs.Bagger_Image(topic_name='image_mono'),
                   camera_info=ecto_sensor_msgs.Bagger_CameraInfo(topic_name='camera_info'),
                   point_cloud=ecto_sensor_msgs.Bagger_PointCloud2(topic_name='points'),
                   )
    im2mat_rgb = ecto_ros.Image2Mat()
    camera_info_conversion = ecto_ros.CameraInfo2Cv()
    point_cloud_conversion = ecto_pcl_ros.Message2PointCloud(format=ecto_pcl.XYZRGB)
    point_cloud_conversion2 = ecto_pcl.PointCloud2PointCloudT(format=ecto_pcl.XYZRGB)

    if options.bag:
        bag_reader = ecto_ros.BagReader('Bag Reader',
                                    baggers=baggers,
                                    bag=options.bag,
                                  )
    else:
        bag_reader = ecto_ros.BagReader('Bag Reader',
                                    baggers=baggers,
                                    bag="/home/vrabaud/tod_data/test_data/Willow_Final_Test_Set/T_01.bag",
                                  )
        

    # connect the visualization
    #image_view = highgui.imshow(name="RGB", waitKey=1000, autoSize=True)
    #mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
    #depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
    #plasm.connect(db_reader['image'] >> image_view['input'],
    #              db_reader['mask'] >> mask_view['input'],
    #              db_reader['depth'] >> depth_view['input'])

    # connect to the model computation
    plasm = ecto.Plasm()
    plasm.connect(bag_reader['image'] >> im2mat_rgb['image'],
                  bag_reader['camera_info'] >> camera_info_conversion['camera_info'],
                  bag_reader['point_cloud'] >> point_cloud_conversion['input'])
    
    #
    json_params = json.loads(str(open(options.config_file).read()))
    feature_descriptor_json_params = str(json_params['feature_descriptor']).replace("'", '"').replace('u"', '"').replace('{u', '{')
    db_json_params = str(json_params['db']).replace("'", '"').replace('u"', '"').replace('{u', '{')
    object_ids = eval(str(json_params['object_ids']).replace("'", '"').replace('u"', '"').replace('{u', '{'))
    guess_json_params = str(json_params['guess']).replace("'", '"').replace('u"', '"').replace('{u', '{')
    search_json_params = str(json_params['search']).replace("'", '"').replace('u"', '"').replace('{u', '{')

    tod_detection = TodDetection(plasm, feature_descriptor_json_params, db_json_params, object_ids, search_json_params,
                                 guess_json_params)
    plasm.connect(im2mat_rgb['image'] >> tod_detection['image'],
                  point_cloud_conversion['output'] >> point_cloud_conversion2['input'],
                  point_cloud_conversion2['output'] >> tod_detection['point_cloud'])

    # write data back to a file
    guess_writer = tod.GuessCsvWriter()
    plasm.connect(tod_detection['object_ids'] >> guess_writer['object_ids'],
                  tod_detection['poses'] >> guess_writer['poses'])

    # send data back to the API

    # display DEBUG data if needed
    if DEBUG:
        print plasm.viz()
        ecto.view_plasm(plasm)

    # execute the pipeline
    plasm.execute()
