import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer, MatPrinter, MatReader, imread
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.features2d import FAST, ORB, DrawKeypoints, Matcher, MatchRefinement, \
    MatchRefinementHSvd, MatchRefinement3d, MatchRefinementPnP, DrawMatches, KeypointsToMat
from ecto_opencv.calib import LatchMat, Select3d, Select3dRegion, PlaneFitter, PoseDrawer, DepthValidDraw, TransformCompose
from ecto_object_recognition.tod_detection import LSHMatcher

class FeatureFinder(ecto.BlackBox):
    orb = ORB
    fast = FAST
    keypointsTo2d = KeypointsToMat
    select3d = Select3d
    image = ecto.Passthrough

    def declare_params(self, p):
        p.declare('use_fast', 'Use fast or not.', False)
        p.forward_all('orb')
        p.forward_all('fast')

    def declare_io(self, p, i, o):
        if p.use_fast:
            i.forward('mask', 'fast')
            i.forward('image', 'image', 'in')
        else:
            i.forward_all('orb')
        self.use_fast = p.use_fast
        i.forward('points3d', 'select3d')
        o.forward_all('orb')
        o.forward_all('keypointsTo2d')
        o.forward_all('select3d')

    def connections(self):
        graph = [self.orb['keypoints'] >> self.keypointsTo2d['keypoints'],
                self.keypointsTo2d['points'] >> self.select3d['points'], ]
        if self.use_fast:
            graph += [self.image[:] >> (self.fast['image'], self.orb['image']),
                      self.fast['keypoints'] >> self.orb['keypoints'],
                      ]
        return graph

class TemplateLoader(ecto.BlackBox):
    points = MatReader
    points3d = MatReader
    descriptors = MatReader
    R = MatReader
    T = MatReader
    image = imread

    def declare_params(self, p):
        p.declare('directory', 'The directory of the template.', '.')

    def declare_io(self, p, i, o):
        for x in ('points', 'points3d', 'descriptors', 'R', 'T'):
            o.forward(x, x, 'mat')
        o.forward('image', 'image')

    def configure(self, p, i, o):
        self.points = MatReader(filename='%s/points.yaml' % p.directory)
        self.points3d = MatReader(filename='%s/points3d.yaml' % p.directory)
        self.descriptors = MatReader(filename='%s/descriptors.yaml' % p.directory)
        self.R = MatReader(filename='%s/R.yaml' % p.directory)
        self.T = MatReader(filename='%s/T.yaml' % p.directory)
        self.image = imread(image_file='%s/train.png' % p.directory)

    def connections(self):
        return [self.points, self.points3d, self.descriptors, self.R, self.T, self.image
                ]

class PlaneEstimator(ecto.BlackBox):
    #find a plane in the center region of the image.
    region = Select3dRegion
    plane_fitter = PlaneFitter
    flag = ecto.Passthrough

    def declare_params(self, p):
        p.forward_all('region')

    def declare_io(self, p, i, o):
        i.forward_all('region')
        i.forward('set', 'flag', cell_key='in')
        o.forward_all('plane_fitter')

    def connections(self):
        return [ self.region['points3d'] >> self.plane_fitter['points3d'],
                ]


class OrbPoseEstimator(ecto.BlackBox):
    '''Estimates the pose of an ORB based template.
    '''

    def declare_params(self, p):
        p.declare('directory', 'The template directory.', '.')
        p.declare('show_matches', 'Display the matches.', False)
    def declare_io(self, p, i, o):
        self.gray_image = ecto.Passthrough('gray Input')
        self.rgb_image = ecto.Passthrough('rgb Input')
        self.K = ecto.Passthrough('K')
        self.points3d = ecto.Passthrough('points3d')
        self.depth_mask = ecto.Passthrough('mask')
        self.pose_estimation = MatchRefinementHSvd('Pose Estimation',reprojection_error=3)
        self.fps = FPSDrawer('FPS')
        self.tr = TransformCompose('Transform Composition')

        #inputs
        i.declare('K', self.K.inputs.at('in'))
        i.declare('image', self.gray_image.inputs.at('in'))
        i.declare('color_image', self.rgb_image.inputs.at('in'))
        i.declare('mask', self.depth_mask.inputs.at('in'))
        i.declare('points3d', self.points3d.inputs.at('in'))

        #outputs
        o.declare('R', self.tr.outputs.at('R'))
        o.declare('T', self.tr.outputs.at('T'))
        o.declare('found', self.pose_estimation.outputs.at('found'))
        o.declare('debug_image', self.fps.outputs.at('image'))
    def configure(self, p, i, o):
        self.train = TemplateLoader(directory=p.directory)
        self.show_matches = p.show_matches
    def connections(self):
        n_features = 4000
        train = self.train
        orb = FeatureFinder('ORB test', n_features=n_features, n_levels=3, scale_factor=1.1, thresh=100, use_fast=False)
        graph = [ self.gray_image[:] >> orb['image'],
                  self.points3d[:] >> orb['points3d'],
                  self.depth_mask[:] >> orb['mask']
                ]

        matcher = LSHMatcher('LSH', n_tables=4, multi_probe_level=1, key_size=10, radius=70)
        #matcher = Matcher()
        graph += [ orb['descriptors'] >> matcher['test'],
                   train['descriptors'] >> matcher['train'],
                  ]

        #3d estimation
        pose_estimation = self.pose_estimation
        graph += [matcher['matches'] >> pose_estimation['matches'],
                  orb['points'] >> pose_estimation['test_2d'],
                  train['points'] >> pose_estimation['train_2d'],
                  orb['points3d'] >> pose_estimation['test_3d'],
                  train['points3d'] >> pose_estimation['train_3d'],
                  ]

        if self.show_matches:
            #display matches
            match_drawer = DrawMatches()
            graph += [pose_estimation['matches'] >> match_drawer['matches'],
                      pose_estimation['matches_mask'] >> match_drawer['matches_mask'],
                      orb['points'] >> match_drawer['test'],
                      train['points'] >> match_drawer['train'],
                      self.rgb_image[:] >> match_drawer['test_image'],
                      train['image'] >> match_drawer['train_image'],
                      match_drawer['output'] >> imshow(name='Matches')['']
                      ]

        tr = self.tr
        fps = self.fps
        pose_draw = PoseDrawer()
        graph += [train['R', 'T'] >> tr['R1', 'T1'],
                  pose_estimation['R', 'T'] >> tr['R2', 'T2'],
                  tr['R', 'T'] >> pose_draw['R', 'T'],
                  pose_estimation['found'] >> pose_draw['trigger'],
                  self.K[:] >> pose_draw['K'],
                  self.rgb_image[:] >> pose_draw['image'],
                  pose_draw['output'] >> fps[:],
                  ]
        return graph
