#!/usr/bin/env python
import ecto
#import ecto_opencv.cv_bp as opencv
from ecto_opencv import highgui, calib, imgproc, line_mod, cv_bp as cv
import os
debug = True

plasm = ecto.Plasm()  #Constructor for Plasm

bin_color = line_mod.ColorMod(gsize=5,gsig = 2.0)
db_color = line_mod.ColorDebug();
coded_color = highgui.imshow(name="coded_color", waitKey=-1, autoSize=True)
raw_image = highgui.imshow(name="raw image", waitKey= -1, autoSize=True)
highgui_db_color = highgui.imshow(name="db_color", waitKey= -1, autoSize=True)
templ_calc = line_mod.ColorTemplCalc(skipx = 5,skipy=5,hist_type=1)
train = line_mod.TrainColorTempl(acceptance_threshold=0.999,threshold=0.95
                                 )
#this will read all images on the user's Desktop
images = highgui.ImageReader("image reader",path=os.path.expanduser("/home/bradski/code/data/set01/images"))
masks = highgui.ImageReader("mask reader",path=os.path.expanduser("/home/bradski/code/data/set01/masks"))
rgb2gray = imgproc.cvtColor("rgb -> gray",flag=7)

#this is similar to a slide show... Wait forever for a key
image_display = highgui.imshow("image display",name="image", waitKey=-1, autoSize=True)
mask_display = highgui.imshow("mask display", name="mask", waitKey=-1, autoSize=True)
colormod_display = highgui.imshow("mod display",name="mod", waitKey=0, autoSize=True)

plasm.connect(images["out"] >> 
                                (image_display['input'], bin_color["image"]),
              masks["out"] >> 
                              rgb2gray[:],
              rgb2gray[:] >>
                              (mask_display['input'], templ_calc['mask']),
              bin_color[:] >> 
                              (db_color[:],templ_calc['colorord']),
              templ_calc[:] >>
                              train[:],
              db_color[:] >> 
                             colormod_display[:],
              )

if debug:
    ecto.view_plasm(plasm)

while(image_display.outputs.out not in (ord('q'),27) ):
    x = plasm.execute()
    print masks.outputs.out.type()
    print cv.CV_8UC1
    print cv.CV_8UC3
    if x :
        break;
    

