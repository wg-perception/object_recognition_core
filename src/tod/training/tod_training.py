#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, features2d
from optparse import OptionParser
import json
import os
import string
import sys
import time
from ecto_object_recognition import capture, tod, tod_training
from object_recognition.tod import FeatureDescriptor

DEBUG = False
DISPLAY = False

class TodModelComputation(ecto.BlackBox):
    def __init__(self, plasm, feature_descriptor_params):
        ecto.BlackBox.__init__(self, plasm)
        self._feature_descriptor_params = feature_descriptor_params
        self.feature_descriptor = FeatureDescriptor(feature_descriptor_params)
        self.twoDToThreeD = tod.TwoDToThreeD()
        self.cameraToWorld = tod.CameraToWorld()
        self._model_stacker = tod_training.TodModelStacker()

    def expose_inputs(self):
        return {'image':self.feature_descriptor['image'],
                'mask':self.feature_descriptor['mask'],
                'depth':self.twoDToThreeD['depth'],
                'K':self.twoDToThreeD['K'],
                'R':self.cameraToWorld['R'],
                'T':self.cameraToWorld['T']}

    def expose_outputs(self):
        return {'points': self._model_stacker['points'],
                'descriptors': self._model_stacker['descriptors']}

    def expose_parameters(self):
        return {'feature_descriptor_params': self._feature_descriptor_params}

    def connections(self):
        return (self.feature_descriptor['keypoints'] >> self.twoDToThreeD['keypoints'],
                self.twoDToThreeD['points'] >> self.cameraToWorld['points'],
                self.cameraToWorld['points'] >> self._model_stacker['points'],
                self.feature_descriptor['descriptors'] >> self._model_stacker['descriptors'])

########################################################################################################################

def parse_options():
    parser = OptionParser()
    parser.add_option("-c", "--config_file", dest="config_file",
                      help='the file containing the configuration as JSON. It should contain the following fields.\n'
                      '"feature_descriptor": with parameters for "combination", "feature" and "descriptor".\n'
                      '"db": parameters about the db: "type", "url".\n'
                      '"objects_ids": the list of object to process, e.g. ["amys_country_cheddar_bowl",'
                      '"band_aid_plastic_strips"]\n'
                      )

    (options, args) = parser.parse_args()
    return options

########################################################################################################################

if __name__ == '__main__':

    options = parse_options()

    # define the input
    if options.config_file is None or not os.path.exists(options.config_file):
        raise 'option file does not exist'

    json_params = json.loads(str(open(options.config_file).read()))
    feature_descriptor_json_params = str(json_params['feature_descriptor']).replace("'", '"').\
                                       replace('u"', '"').replace('{u', '{')
    feature_descriptor_params = eval(feature_descriptor_json_params)

    db_url = str(json_params['db']['url'])
    object_ids = json_params['object_ids']
    for object_id in object_ids:
        object_id = object_id.encode('ascii')
        db_reader = capture.ObservationReader("db_reader", db_url=db_url, object_id=object_id)

        # connect the visualization
        plasm = ecto.Plasm()
        if DISPLAY:
            image_view = highgui.imshow(name="RGB", waitKey=1000, autoSize=True)
            mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
            depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
            plasm.connect(db_reader['image'] >> image_view['input'],
                          db_reader['mask'] >> mask_view['input'],
                          db_reader['depth'] >> depth_view['input'])

        # connect to the model computation
        tod_model = TodModelComputation(plasm, feature_descriptor_params)
        plasm.connect(db_reader['image', 'mask', 'depth', 'K', 'R', 'T'] >> tod_model['image', 'mask', 'depth', 'K', 'R', 'T'])

        # persist to the DB
        db_json_params_str = str(json_params['db']).replace("'", '"').replace('u"', '"').replace('{u', '{')
        _db_writer = tod_training.TodModelInserter("db_writer", collection_models='models',
                                                  db_json_params=db_json_params_str, object_id=object_id,
                                                  model_json_params=feature_descriptor_json_params)
        orb_params = None
        # TODO
        #db_writer.add_misc(orb_params)
        
        # never execute the db_writer
        db_writer = ecto.If(cell=_db_writer)
        db_writer.inputs.__test__ = False
        plasm.connect(tod_model['points', 'descriptors'] >> db_writer['points', 'descriptors'])

        if DEBUG:
            #render the DAG with dot
            print plasm.viz()
            ecto.view_plasm(plasm)

        if DISPLAY:
            while(image_view.outputs.out not in (27, ord('q'))):
                if(plasm.execute(1) != 0): break
        else:
            sched = ecto.schedulers.Singlethreaded(plasm)
            sched.execute()

        _db_writer.process()
