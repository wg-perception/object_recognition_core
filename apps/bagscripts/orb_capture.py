import ecto
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer, MatPrinter
from ecto_opencv.features2d import ORB, DrawKeypoints, Matcher, MatchRefinement, MatchRefinement3d, DrawMatches
from ecto_opencv.imgproc import cvtColor, Conversion
from ecto_opencv.calib import LatchMat, Select3d, Select3dRegion, PlaneFitter, PoseDrawer
from ecto_object_recognition.tod_training import KeypointsToMat


class FeatureFinder(ecto.BlackBox):
    orb = ORB
    keypointsTo2d = KeypointsToMat
    select3d = Select3d

    def declare_params(self, p):
        p.forward_all('orb')

    def declare_io(self, p, i, o):
        i.forward('points3d', 'select3d')
        i.forward_all('orb')
        o.forward_all('orb')
        o.forward_all('keypointsTo2d')
        o.forward_all('select3d')

    def connections(self):
        return [self.orb['keypoints'] >> self.keypointsTo2d['keypoints'],
                self.keypointsTo2d['points'] >> self.select3d['points'],
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
        i.forward('set','flag',cell_key='in')
        o.forward_all('plane_fitter')

    def connections(self):
        return [ self.region['points3d'] >> self.plane_fitter['points3d'],
                #self.plane_fitter['R'] >> MatPrinter(name='R')[:],
                #self.plane_fitter['T'] >> MatPrinter(name='T')[:],
                ]