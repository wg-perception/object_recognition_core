#!/usr/bin/env python
# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc

class PoseFromFiducial(ecto.BlackBox):
    def __init__(self, plasm, rows, cols, pattern_type, square_size, offset_x=0, offset_y=0, offset_z=0, debug=True):
        ecto.BlackBox.__init__(self, plasm)
        self.video_cap = ecto.Passthrough('Image Input')
        self.camera_info = ecto.Passthrough('K')

        self.circle_detector = calib.PatternDetector('Dot Detector',
                                                rows=rows, cols=cols,
                                                pattern_type=pattern_type,
                                                square_size=square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y,
                                                offset_z=offset_z)
        self.pose_calc = calib.FiducialPoseFinder('Pose Calc')
        self.debug = debug
        self.circle_drawer = calib.PatternDrawer('Circle Draw',
                                                     rows=rows, cols=cols)
        self.pose_draw = calib.PoseDrawer('Pose Draw')
    def expose_outputs(self):
        return {
                'R': self.pose_calc['R'],
                'T': self.pose_calc['T'],
                'found':  self.circle_detector['found'],
                'points': self.circle_detector['out'],
                'ideal': self.circle_detector['out'],
                'debug_image':  self.pose_draw['output'],
               }
    def expose_inputs(self):
        return {
                'image': self.video_cap[:],
                'color_image': self.circle_drawer['input'],
                'K': self.camera_info[:]
               }
    def expose_parameters(self):
        return {
                }
    def connections(self):
        graph = [self.video_cap[:] >> self.circle_detector['input'],
                 self.camera_info[:] >> self.pose_calc['K'],
                 self.circle_detector['out', 'ideal', 'found'] >> self.pose_calc['points', 'ideal', 'found'],
                 self.circle_drawer['out'] >> self.pose_draw['image'],
                 self.pose_calc['R', 'T'] >> self.pose_draw['R', 'T'],
                 self.circle_detector['out', 'found'] >> self.circle_drawer['points', 'found'],
                 self.camera_info[:] >> self.pose_draw['K'],
               ]
        return graph

class OpposingDotPoseEstimator(ecto.BlackBox):
    def __init__(self, plasm, rows, cols, pattern_type, square_size, debug=True):
        ecto.BlackBox.__init__(self, plasm)
        self.gray_image = ecto.Passthrough('gray Input')
        self.rgb_image = ecto.Passthrough('rgb Input')
        self.camera_info = ecto.Passthrough('K')
         
        self.gather = calib.GatherPoints("gather", N=2)
        self.invert = imgproc.BitwiseNot()
        self.debug = debug
        offset_x = -.310
        offset_y = -.101099
        self.cd_bw = calib.PatternDetector('Dot Detector, B/W',
                                                rows=rows, cols=cols,
                                                pattern_type=pattern_type,
                                                square_size=square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y,
                                                )
        offset_x = .150
        self.cd_wb = calib.PatternDetector('Dot Detector, W/B',
                                                rows=rows, cols=cols,
                                                pattern_type=pattern_type,
                                                square_size=square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y,
                                                )
        self.pose_calc = calib.FiducialPoseFinder('Pose Calc')
        self.circle_drawer = calib.PatternDrawer('Circle Draw',
                                                 rows=rows, cols=cols)
        self.circle_drawer2 = calib.PatternDrawer('Circle Draw',
                                                 rows=rows, cols=cols)
        self.pose_draw = calib.PoseDrawer('Pose Draw')
    def expose_outputs(self):
        outputs = {
                'R':  self.pose_calc['R'],
                'T':  self.pose_calc['T'],
                'points':self.gather['out'],
                'ideal': self.gather['ideal'],
                'found': self.gather['found']
               }
        if self.debug:
            outputs['debug_image'] = self.pose_draw['output']
        return outputs
    
    def expose_inputs(self):
        return {
                'image': self.gray_image[:],
                'color_image': self.rgb_image[:],
                'K': self.camera_info[:]
               }
    def expose_parameters(self):
        return {
                }
    def connections(self):
        graph = [
                self.gray_image[:] >> (self.invert[:], self.cd_bw['input']),
                self.cd_bw['found', 'ideal', 'out'] >> self.gather['found_0000', 'ideal_0000', 'points_0000'],
                self.cd_wb['found', 'ideal', 'out'] >> self.gather['found_0001', 'ideal_0001', 'points_0001'],
                self.invert[:] >> self.cd_wb['input'],
                self.gather['out', 'ideal', 'found'] >> self.pose_calc['points', 'ideal', 'found'],
                self.camera_info[:] >> self.pose_calc['K'],
                ]
        graph += ([
                      self.rgb_image[:] >> self.circle_drawer['input'],
                      self.circle_drawer['out'] >> self.pose_draw['image'],
                      self.pose_calc['R', 'T'] >> self.pose_draw['R', 'T'],
                      self.gather['out', 'found'] >> self.circle_drawer['points', 'found'],
                      self.gather['found'] >> self.pose_draw['trigger'],
                      self.camera_info[:] >> self.pose_draw['K'], 
                     ])
        return graph


    
    
if "__main__" == __name__:
    import sys
    
    plasm = ecto.Plasm()
    sched = ecto.schedulers.Singlethreaded(plasm)
    
    #lil bit of debug On/Off
    debug = True
    if 'R' in sys.argv:
        debug = False
    camera_info = calib.CameraIntrinsics('Camera Info',
                                                  camera_file="camera.yml")
    #add our black box to the plasm.
    poser = OpposingDotPoseEstimator(plasm,
                                        rows=5, cols=3,
                                        pattern_type="acircles",
                                        square_size=0.04, debug=debug)
    video_cap = highgui.VideoCapture(video_device=0)
    rgb2gray = imgproc.cvtColor('rgb -> gray', flag=7)
    display = highgui.imshow('Poses', name='Poses', waitKey=2, autoSize=True)
    imsaver = highgui.ImageSaver('pose image saver', filename='image_pose_')
    rawsaver = highgui.ImageSaver('raw image saver', filename='image_raw_')

    plasm.connect(video_cap['image'] >> (rgb2gray[:], poser['color_image'], rawsaver['image']),
                  rgb2gray[:] >> poser['image'],
                  poser['debug_image'] >> (display['input'], imsaver['image']),
                  display['out'] >> (imsaver['trigger'], rawsaver['trigger']),
                  camera_info['K'] >> (poser['K']),
                  )
                  
    ecto.view_plasm(plasm)
    with open('blackbox_within_a_blackbox.viz', 'w') as viz:
        viz.write(plasm.viz())
    sched.execute()
