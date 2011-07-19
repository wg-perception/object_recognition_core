#!/bin/python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc
import tod_db
import time
debug = True 
plasm = ecto.Plasm()


image_view = highgui.imshow(name="RGB", waitKey=5, autoSize=True)
mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
db_reader = tod_db.ObservationReader("db_reader", object_id="object_1311046926")

plasm.connect(db_reader, "image", image_view, "input")
plasm.connect(db_reader, "mask", mask_view, "input")
plasm.connect(db_reader, "depth", depth_view, "input")

sched = ecto.schedulers.Singlethreaded(plasm)
sched.execute()
