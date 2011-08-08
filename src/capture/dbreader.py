#!/usr/bin/env python

import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc
import capture
import time
debug = True
while True:
    plasm = ecto.Plasm()
    
    image_view = highgui.imshow(name="RGB", waitKey=5, autoSize=True)
    mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
    depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
    db_reader = capture.ObservationReader("db_reader", object_id="6ea81529b747a2e8148389481200de19")
    
    plasm.connect(db_reader, "image", image_view, "input")
    plasm.connect(db_reader, "mask", mask_view, "input")
    plasm.connect(db_reader, "depth", depth_view, "input")
    
    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()
