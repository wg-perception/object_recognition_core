#!/usr/bin/env python
import ecto
import ecto_ros
import ecto_pcl
import ecto_pcl_ros
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, features2d
import ecto_sensor_msgs
from optparse import OptionParser
import time
import tod_db
import tod

DEBUG = True

class TodDetection(ecto.BlackBox):
    def __init__(self, plasm, feature_descriptor_params_file):
        ecto.BlackBox.__init__(self, plasm)
        if not feature_descriptor_params_file:
            feature_descriptor_params_file = ''

        self._feature_descriptor_params_file = feature_descriptor_params_file
        self.feature_descriptor = features2d.FeatureDescriptor(param_file=feature_descriptor_params_file)
        self.descriptor_matcher = tod.DescriptorMatcher()
        if feature_descriptor_params_file:
            self.guess_generator = tod.GuessGenerator(config_file=feature_descriptor_params_file)
        else:
            self.guess_generator = tod.GuessGenerator()

    def expose_inputs(self):
        return {'image':self.feature_descriptor['image'],
                'mask':self.feature_descriptor['mask'],
                'point_cloud':self.guess_generator['point_cloud']}

    def expose_outputs(self):
        return {'object_ids': self.guess_generator['object_ids'],
                'poses': self.guess_generator['poses']}

    def expose_parameters(self):
        return {'feature_descriptor_params': self._feature_descriptor_params_file}

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
    baggers = dict(image=ecto_sensor_msgs.Bagger_Image(topic_name='image_mono'),
                   camera_info=ecto_sensor_msgs.Bagger_CameraInfo(topic_name='camera_info'),
                   point_cloud=ecto_sensor_msgs.Bagger_PointCloud2(topic_name='points'),
                   )
    im2mat_rgb = ecto_ros.Image2Mat()
    camera_info_conversion = ecto_ros.CameraInfo2Cv()
    point_cloud_conversion = ecto_pcl_ros.Message2PointCloud(format=ecto_pcl.XYZRGB)

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
    tod_detection = TodDetection(plasm, options.config_file)
    plasm.connect(im2mat_rgb['image'] >> tod_detection['image'])#,
                  #point_cloud_conversion['output'] >> tod_detection['point_cloud'])

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
    while True:
        if(plasm.execute(1) != 0):
            break
