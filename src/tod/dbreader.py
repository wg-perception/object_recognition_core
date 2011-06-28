#!/bin/python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, tod, objcog_db
import time
debug = True 
plasm = ecto.Plasm()


image_view = highgui.imshow(name="RGB", waitKey=1000, autoSize=True)
mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
db_reader = objcog_db.ObservationReader("db_reader", object_id="object_01")

plasm.connect(db_reader, "image", image_view, "input")
plasm.connect(db_reader, "mask", mask_view, "input")
plasm.connect(db_reader, "depth", depth_view, "input")

if debug:
  print plasm.viz()
  ecto.view_plasm(plasm)

    
while(image_view.outputs.out not in (27, ord('q'))):
    if(plasm.execute(1) != 0): break
    
