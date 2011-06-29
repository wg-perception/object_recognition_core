#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, tod, objcog_db, features2d
import time
debug = True
plasm = ecto.Plasm()

class TodModelComputation(ecto.BlackBox):
    def __init__(self, plasm, orb_params = None):
        ecto.BlackBox.__init__(self, plasm)
        self._orb_params = orb_params
        self.orb = features2d.ORB()
        self.twoDToThreeD = tod.TwoDToThreeD()
        self.cameraToWorld = tod.CameraToWorld()

    def expose_inputs(self):
        return {'image':self.orb['image'],
                'mask':self.orb['mask'],
                'depth':self.twoDToThreeD['depth'],
                'K':self.twoDToThreeD['K'],
                'R':self.cameraToWorld['R'],
                'T':self.cameraToWorld['T']}

    def expose_outputs(self):
        return {'points': self.cameraToWorld['points'],
                'descriptors': self.orb['descriptors']}

    def expose_parameters(self):
        return {'descriptor_param': self._orb_params}

    def connections(self):
        return (self.orb['kpts'] >> self.twoDToThreeD['keypoints'],
                self.twoDToThreeD['points'] >> self.cameraToWorld['points'])

# define the input
db_reader = objcog_db.ObservationReader("db_reader", object_id="object_01")

# connect the visualization
image_view = highgui.imshow(name="RGB", waitKey=1000, autoSize=True)
mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
plasm.connect(db_reader['image'] >> image_view['input'],
              db_reader['mask'] >> mask_view['input'],
              db_reader['depth'] >> depth_view['input'])

# connect to the model computation
tod_model = TodModelComputation(plasm)
plasm.connect(db_reader['image', 'mask', 'depth', 'K', 'R', 'T'] >> tod_model['image', 'mask', 'depth', 'K', 'R', 'T'])

# persist to the DB
db_writer = objcog_db.TodModelInserter("db_writer", object_id="object_01")
orb_params = None
#db_writer.add_misc(orb_params)
plasm.connect(tod_model['points', 'descriptors'] >> db_writer['points', 'descriptors'])

if debug:
    print plasm.viz()
    ecto.view_plasm(plasm)

while(image_view.outputs.out not in (27, ord('q'))):
    if(plasm.execute(1) != 0): break
