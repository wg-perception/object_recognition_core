import ecto
from ecto_opencv import highgui, calib, imgproc

class OpposingDotPoseEstimator(ecto.BlackBox):
    '''Estimates the pose of a fiducial that has a black on white pattern and a white on black pattern.
    TODO support other configurations...
    '''
    def declare_params(self, p):
        p.declare('rows', 'Number of rows in the pattern.', 11)
        p.declare('cols', 'Number of cols in the pattern.', 7)
        p.declare('pattern_type', 'Type of pattern', calib.ASYMMETRIC_CIRCLES_GRID)
        p.declare('square_size', 'The pattern spacing', 0.1)
        p.declare('debug', 'Debug the detector.', True)

    def declare_io(self, p, i, o):
        self.gray_image = ecto.Passthrough('gray Input')
        self.rgb_image = ecto.Passthrough('rgb Input')
        self.camera_info = ecto.Passthrough('K')
        self.gather = calib.GatherPoints("gather", N=2)
        #TODO parameterize the quantizer
        #abuse saturated Arithmetics http://opencv.itseez.com/modules/core/doc/intro.html?highlight=saturated.
        self.quantizer = imgproc.Quantize('Quantizer', alpha=1, beta=0)
        self.invert = imgproc.BitwiseNot()
        self.debug = p.debug
        offset_x = -.3095 #TODO: FIXME hard coded
        offset_y = -.1005
        self.cd_bw = calib.PatternDetector('Dot Detector, B/W',
                                                rows=p.rows, cols=p.cols,
                                                pattern_type=p.pattern_type,
                                                square_size=p.square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y,
                                                )
        offset_x = .1505 #TODO: FIXME hard coded
        self.cd_wb = calib.PatternDetector('Dot Detector, W/B',
                                                rows=p.rows, cols=p.cols,
                                                pattern_type=p.pattern_type,
                                                square_size=p.square_size,
                                                offset_x=offset_x,
                                                offset_y=offset_y,
                                                )
        self.pose_calc = calib.FiducialPoseFinder('Pose Calc')
        self.circle_drawer = calib.PatternDrawer('Circle Draw',
                                                 rows=p.rows, cols=p.cols)
        self.circle_drawer2 = calib.PatternDrawer('Circle Draw',
                                                 rows=p.rows, cols=p.cols)
        self.pose_draw = calib.PoseDrawer('Pose Draw')
        self.fps = highgui.FPSDrawer()

        #inputs
        i.declare('image', self.gray_image.inputs.at('in'))
        i.declare('color_image', self.rgb_image.inputs.at('in'))
        i.declare('K', self.camera_info.inputs.at('in'))

        #outputs
        o.declare('R', self.pose_calc.outputs.at('R'))
        o.declare('T', self.pose_calc.outputs.at('T'))
        o.declare('found', self.gather.outputs.at('found'))
        o.declare('debug_image', self.fps.outputs.at('image'))

    def connections(self):
        graph = [
                self.gray_image[:] >> self.quantizer[:],
                self.quantizer[:] >> (self.invert[:], self.cd_bw['input']),
                self.cd_bw['found', 'ideal', 'out'] >> self.gather['found_0000', 'ideal_0000', 'points_0000'],
                self.cd_wb['found', 'ideal', 'out'] >> self.gather['found_0001', 'ideal_0001', 'points_0001'],
                self.invert[:] >> self.cd_wb['input'],
                self.gather['out', 'ideal', 'found'] >> self.pose_calc['points', 'ideal', 'found'],
                self.camera_info[:] >> self.pose_calc['K'],
                ]
        if self.debug:
            graph += [self.rgb_image[:] >> self.circle_drawer['input'],
                      self.circle_drawer2[:] >> self.pose_draw['image'],
                      self.pose_calc['R', 'T'] >> self.pose_draw['R', 'T'],
                      self.circle_drawer[:] >> self.circle_drawer2['input'],
                      self.cd_bw['out', 'found'] >> self.circle_drawer['points', 'found'],
                      self.cd_wb['out', 'found'] >> self.circle_drawer2['points', 'found'],
                      self.gather['found'] >> self.pose_draw['trigger'],
                      self.camera_info[:] >> self.pose_draw['K'],
                      self.pose_draw['output'] >> self.fps[:],
#                      self.quantizer[:] >> highgui.imshow(name='quantized')[:],
             ]
        return graph
