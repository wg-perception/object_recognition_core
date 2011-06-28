#!/bin/python
import ecto
from ecto_opencv import highgui, cv_bp as opencv, calib, imgproc, tod, objcog_db
import time
#import orb as imgproc

debug = True

class PoseEstimator:
    
    def __init__(self):
        self.draw_debug = debug
        self.circle_drawer = calib.PatternDrawer("Found Pattern Drawer", rows=7, cols=3)
        self.pattern_show = highgui.imshow("Pattern view", name="pattern", waitKey= -1, autoSize=True)
        self.pose_drawer = calib.PoseDrawer("pattern pose")            
        self.rgb2gray = imgproc.cvtColor("rgb -> gray", flag=7)
        self.circle_detector = calib.PatternDetector("asymmetric dot detector", rows=7, cols=3, pattern_type="acircles", square_size=0.03)
        self.poser = calib.FiducialPoseFinder("Pose Estimation")
        self.camera_intrinsics = calib.CameraIntrinsics("camera.yml", camera_file="camera.kinect.vga.yml")
        
    def declare_graph(self, plasm, image_source):
        plasm.connect(image_source, "image", self.rgb2gray, "input")
        plasm.connect(self.rgb2gray, "out", self.circle_detector, "input")
        plasm.connect(self.camera_intrinsics, "K", self.poser, "K")
        plasm.connect(self.circle_detector, "out", self.poser, "points")
        plasm.connect(self.circle_detector, "ideal", self.poser, "ideal")
        plasm.connect(self.circle_detector, "found", self.poser, "found")
        if self.draw_debug:
            plasm.connect(image_source, "image", self.circle_drawer, "input")
            plasm.connect(self.circle_detector, "out", self.circle_drawer, "points")
            plasm.connect(self.circle_detector, "found", self.circle_drawer, "found")
        plasm.connect(self.poser, "R", self.pose_drawer, "R")
        plasm.connect(self.poser, "T", self.pose_drawer, "T")
        plasm.connect(self.circle_drawer, "out", self.pose_drawer, "image")
        plasm.connect(self.camera_intrinsics, "K", self.pose_drawer, "K")
        plasm.connect(self.pose_drawer, "output", self.pattern_show, "input")
        
plasm = ecto.Plasm()

pose_est = PoseEstimator()

capture = highgui.OpenNICapture(video_mode=opencv.CV_CAP_OPENNI_VGA_30HZ)
image_view = highgui.imshow(name="RGB", waitKey=10, autoSize=True)
mask_view = highgui.imshow(name="mask", waitKey= -1, autoSize=True)
object_view = highgui.imshow("object_view", name="object_view", waitKey= -1, autoSize=True)

masker = tod.PlanarSegmentation(z_min=0)
if debug:
    depth_view = highgui.imshow(name="Depth", waitKey= -1, autoSize=True);
    plasm.connect(capture, "image", image_view , "input")
    plasm.connect(capture, "depth", depth_view , "input")
    
pose_est.declare_graph(plasm, capture)

plasm.connect(pose_est.poser, "R", masker, "R")
plasm.connect(pose_est.poser, "T", masker, "T")
plasm.connect(pose_est.camera_intrinsics, "K", masker, "K")
plasm.connect(capture, "depth", masker , "depth")
bitwise_and = imgproc.BitwiseAnd()
bitwise_and2 = imgproc.BitwiseAnd("image_ander")

#plasm.connect(masker,"mask",bitwise_and,"a")
#plasm.connect(capture,"valid",bitwise_and,"b")
#plasm.connect(bitwise_and,"out", mask_view,"input")

#plasm.connect(bitwise_and,"out",bitwise_and2,"a")
plasm.connect(masker, "mask", bitwise_and2, "a")

plasm.connect(pose_est.rgb2gray, "out", bitwise_and2, "b")
plasm.connect(bitwise_and2, "out", object_view, "input")

db_inserter = objcog_db.ObservationInserter("db_inserter", object_id="object_01")
plasm.connect(pose_est.poser, "R", db_inserter, "R")
plasm.connect(pose_est.poser, "T", db_inserter, "T")
plasm.connect(pose_est.camera_intrinsics, "K", db_inserter, "K")
plasm.connect(image_view, "out", db_inserter, "trigger")
plasm.connect(capture, "image", db_inserter , "image")
plasm.connect(capture, "depth", db_inserter , "depth")
plasm.connect(masker, "mask", db_inserter , "mask")

if debug:
  print plasm.viz()
  ecto.view_plasm(plasm)

    
while(image_view.outputs.out not in (27, ord('q'))):
    plasm.execute(1)
    
