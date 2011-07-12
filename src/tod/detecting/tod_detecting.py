#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, features2d
from optparse import OptionParser
import time
import tod
import tod_db

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
            self.guessGenerator = tod.GuessGenerator(config_file=feature_descriptor_params_file)
        else:
            self.guessGenerator = tod.GuessGenerator()

    def expose_inputs(self):
        return {'image':self.feature_descriptor['image'],
                'mask':self.feature_descriptor['mask'],
                'point_cloud':self.guessGenerator['point_cloud']}

    def expose_outputs(self):
        return {'guesses': self.guessGenerator['guesses']}

    def expose_parameters(self):
        return {'feature_descriptor_params': self._feature_descriptor_params_file}

    def connections(self):
        return (self.feature_descriptor['keypoints'] >> self.guessGenerator['keypoints'],
                self.feature_descriptor['descriptors'] >> self.descriptor_matcher['descriptors'],
                self.descriptor_matcher['matches'] >> self.guessGenerator['matches'],
                self.descriptor_matcher['matches_3d'] >> self.guessGenerator['matches_3d'])

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
    if options.bag:
        bag_reader = tod.BagReader(path=options.bag)
    else:
        bag_reader = tod.BagReader(path="/home/vrabaud/tod_data/test_data/Willow_Final_Test_Set/T_01.bag")

    # connect the visualization
    #image_view = highgui.imshow(name="RGB", waitKey=1000, autoSize=True)
    #mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
    #depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
    #plasm.connect(db_reader['image'] >> image_view['input'],
    #              db_reader['mask'] >> mask_view['input'],
    #              db_reader['depth'] >> depth_view['input'])

    # connect to the model computation
    plasm = ecto.Plasm()
    tod_detection = TodDetection(plasm, options.config_file)
    plasm.connect(bag_reader['image', 'point_cloud'] >> tod_detection['image', 'point_cloud'])

    # write data back to a file
    guess_writer = tod.GuessWriter()
    plasm.connect(tod_detection['guesses'] >> guess_writer['guesses'])

    # send data back to the API

    # display DEBUG data if needed
    if DEBUG:
        print plasm.viz()
        ecto.view_plasm(plasm)

    # execute the pipeline
    while True:
        if(plasm.execute(1) != 0):
            break
