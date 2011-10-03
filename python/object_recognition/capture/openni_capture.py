import ecto
from ecto_opencv import highgui, calib, imgproc, cv_bp as cv
import ecto_ros, ecto_sensor_msgs, ecto_geometry_msgs
from ecto_object_recognition import capture
from fiducial_pose_est import OpposingDotPoseEstimator

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo
PoseBagger = ecto_geometry_msgs.Bagger_PoseStamped

def xtion_highres(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.SXGA_RES,
                   depth_resolution=ResolutionMode.VGA_RES,
                   rgb_fps=30, depth_fps=30,
                   device_number=device_n,
                   registration=True,
                   synchronize=True,
                   device=Device.ASUS_XTION_PRO_LIVE
                   )

from arbotix import *
import math
import time
from ecto_opencv import imgproc, calib, highgui

def find_diff(prev, current):
  max_val = 0xFFFF #16bit max_val
  min_val = 0x0
  if current - prev >= 0:
    return current - prev
  else:
    return (max_val - prev) + current

def rotate(a, MY_SERVO, degrees, speed):
  a.enableWheelMode(MY_SERVO)
  prev = pos = a.getPosition(MY_SERVO)
  ticks_per_degree = 15.170
  total_ticks = ticks_per_degree * degrees
  # set speed for rotation in joint mode
  a.setSpeed(MY_SERVO, speed)    # half speed, values between 0 and 1023
  diff = 0
  while diff < total_ticks:
      pos = a.getPosition(MY_SERVO)
      diff = find_diff(prev, pos)
      time.sleep(0.001)
  a.setSpeed(MY_SERVO, 0)
  time.sleep(0.025)

class TurnTable(ecto.Cell):
    '''Uses the arbotix library to talk to servoes.'''

    @staticmethod
    def declare_params(params):
        params.declare('angle_thresh', 'Angular threshold', math.pi / 36)

    @staticmethod
    def declare_io(_p, _i, o):
        o.declare('trigger', 'Capture', True)

    def configure(self, params):
        try:
            self.MY_SERVO = 0xE9
            self.a = ArbotiX('/dev/ttyUSB0', baud=1e6) #1 meg for e
            self.a.disableTorque(self.MY_SERVO)
            self.a.disableWheelMode(self.MY_SERVO, resolution=12)
            pos = self.a.getPosition(self.MY_SERVO)
            self.a.setSpeed(self.MY_SERVO, 100)
            self.a.setPosition(self.MY_SERVO, 0)
            while self.a.getPosition(self.MY_SERVO) > 5:
                time.sleep(0.1)
            print 'At position: ', self.a.setPosition(self.MY_SERVO, 0)
            self.settle_count = -1 #use to alternate movement
            self.total = 0
            #print params
            self.delta_angle = params.angle_thresh * 180 / math.pi
            self.max_angle = 720
            print self.delta_angle, 'delta angle.'
        except Exception, e:
            print e

    def process(self, _i, outputs):
        if self.settle_count == 0:
            self.settle_count += 1
            self.total += 1
            rotate(self.a, self.MY_SERVO, self.delta_angle, 25)
            print 'Total angular travel :', self.total * self.delta_angle
            outputs.trigger = False
        elif self.settle_count >= 2:
            outputs.trigger = True
            self.settle_count = 0
        else:
            self.settle_count += 1
            time.sleep(0.25)
        if self.total * self.delta_angle > self.max_angle:
            return 1
        return 0

    def __del__(self):
        print 'Stopping servo....'
        a = ArbotiX('/dev/ttyUSB0', baud=1e6)
        MY_SERVO = 0xE9
        a.disableTorque(MY_SERVO)

def create_capture_plasm(bag_name, angle_thresh, z_min=0.01, y_crop=0.10, x_crop=0.10, z_crop=1.0, n_desired=72, preview=False, use_turn_table=True):
    '''
    Creates a plasm that will capture openni data into a bag, using a dot pattern to sparsify views.
    
    @param bag_name: A filename for the bag, will write to this file.
    @param angle_thresh: The angle threshhold in radians to sparsify the views with.  
    '''
    from ecto_ros import Mat2Image, RT2PoseStamped

    plasm = ecto.Plasm()
    graph = []
    subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
                depth=ImageSub(topic_name='/camera/depth_registered/image', queue_size=0),
                image_ci=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
                depth_ci=CameraInfoSub(topic_name='/camera/depth_registered/camera_info', queue_size=0),
                )

    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs
                                 )

    image = ecto_ros.Image2Mat('rgb -> cv::Mat')
    camera_info = ecto_ros.CameraInfo2Cv('camera_info -> cv::Mat')

    poser = OpposingDotPoseEstimator(rows=5, cols=3,
                                     pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                     square_size=0.04, debug=True)

    bgr2rgb = imgproc.cvtColor('rgb -> bgr', flag=imgproc.Conversion.RGB2BGR)
    rgb2gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.Conversion.RGB2GRAY)

    delta_pose = ecto.If('delta R|T', cell=capture.DeltaRT(angle_thresh=angle_thresh,
                                                          n_desired=n_desired))

    display = highgui.imshow(name='Poses')

    poseMsg = RT2PoseStamped(frame_id='/camera_rgb_optical_frame')

    graph += [sync['image'] >> image[:],
              image[:] >> (rgb2gray[:], bgr2rgb[:]),
              bgr2rgb[:] >> poser['color_image'],
              rgb2gray[:] >> poser['image'],
              poser['debug_image'] >> (display['image'],),
              sync['image_ci'] >> camera_info['camera_info'],
              camera_info['K'] >> poser['K'],
              poser['R', 'T', 'found'] >> delta_pose['R', 'T', 'found'],
              poser['R', 'T'] >> poseMsg['R', 'T'],
              ]

    depth = ecto_ros.Image2Mat()
    rescale_depth = capture.RescaledRegisteredDepth() #this is for SXGA mode scale handling.
    segmentation = calib.PlanarSegmentation(z_min=z_min, y_crop=y_crop, x_crop=x_crop, z_crop=z_crop) #do NOT add this to the plasm
    masker = ecto.If('Planar Segmentation', cell=segmentation)
    masker.inputs.__test__ = True
    maskMsg = Mat2Image(frame_id='/camera_rgb_optical_frame')

    graph += [
              sync['depth'] >> depth['image'],
              depth['image'] >> rescale_depth['depth'],
              image[:] >> rescale_depth['image'], #grabs the size from image
              rescale_depth['depth'] >> masker['depth'],
              camera_info['K'] >> masker['K'],
              poser['R', 'T'] >> masker['R', 'T'],
              masker['mask'] >> maskMsg[:],
              ]

    #display the mask
    mask_and = imgproc.BitwiseAnd()
    mask2rgb = imgproc.cvtColor('mask -> rgb', flag=imgproc.Conversion.GRAY2RGB)
    graph += [
              masker['mask'] >> mask2rgb['image'],
              mask2rgb['image'] >> mask_and['a'],
              image[:] >> mask_and['b'],
              mask_and[:] >> highgui.imshow(name='mask')[:],
            ]
    if not preview:
        baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   mask=ImageBagger(topic_name='/camera/mask'),
                   pose=PoseBagger(topic_name='/camera/pose'),
                   image_ci=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
                   depth_ci=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
                   )
        bagwriter = ecto.If('Bag Writer if R|T',
                            cell=ecto_ros.BagWriter(baggers=baggers, bag=bag_name)
                            )
        keys = subs.keys()
        graph += [sync[:] >> bagwriter[keys],
                  poseMsg['pose'] >> bagwriter['pose'],
                  maskMsg[:] >> bagwriter['mask'],
                  ]
        if use_turn_table:
            table = TurnTable(angle_thresh=angle_thresh)
            ander = ecto.And()
            graph += [
                  table['trigger'] >> (delta_pose['__test__'], ander['in2']),
                  delta_pose['novel'] >> ander['in1'],
                  ander['out'] >> bagwriter['__test__']
                  ]
        else:
            #TODO add still filter:
            delta_pose.inputs.__test__ = True
            graph += [
                  delta_pose['novel'] >> bagwriter['__test__']
                  ]
    plasm.connect(graph)
    return (plasm, segmentation) # return segmentation for tuning of parameters.
