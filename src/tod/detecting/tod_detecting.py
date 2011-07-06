#!/usr/bin/env python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, features2d
import tod_db
import tod
import time
debug = True
plasm = ecto.Plasm()

class TodDetection(ecto.BlackBox):
    def __init__(self, plasm, orb_params=None):
        ecto.BlackBox.__init__(self, plasm)
        self._orb_params = orb_params
        self.orb = features2d.ORB()
        self.guessGenerator = tod.GuessGenerator()

    def expose_inputs(self):
        return {'image':self.orb['image'],
                'mask':self.orb['mask'],
                'point_cloud':self.guessGenerator['point_cloud']}

    def expose_outputs(self):
        return {'guesses': self.guessGenerator['guesses']}

    def expose_parameters(self):
        return {'descriptor_param': self._orb_params}

    def connections(self):
        return (self.orb['kpts'] >> self.guessGenerator['keypoints'],
                self.orb['descriptors'] >> self.guessGenerator['descriptors']
                )

# define the input
bag_reader = tod.BagReader(path="/home/vrabaud/tod_data/test_data/Willow_Final_Test_Set/T_01.bag")

# connect the visualization
#image_view = highgui.imshow(name="RGB", waitKey=1000, autoSize=True)
#mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
#depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
#plasm.connect(db_reader['image'] >> image_view['input'],
#              db_reader['mask'] >> mask_view['input'],
#              db_reader['depth'] >> depth_view['input'])

# connect to the model computation
tod_detection = TodDetection(plasm)
plasm.connect(bag_reader['image', 'point_cloud'] >> tod_detection['image', 'point_cloud'])

# send data back to the API
#db_writer = objcog_db.TodModelInserter("db_writer", object_id="object_01")
#orb_params = None
#db_writer.add_misc(orb_params)
#plasm.connect(tod_model['points', 'descriptors'] >> db_writer['points', 'descriptors'])

if debug:
    print plasm.viz()
    ecto.view_plasm(plasm)

while True:
    if(plasm.execute(1) != 0): break

#while(image_view.outputs.out not in (27, ord('q'))):
#    if(plasm.execute(1) != 0): break
