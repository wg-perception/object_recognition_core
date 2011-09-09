# abstract the input.
import ecto
from ecto_opencv import highgui, calib, imgproc, cv_bp as cv
from object_recognition.observations import *
import ecto_ros, ecto_sensor_msgs
from ecto_object_recognition import capture
ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo


def create_capture_plasm(bag_name, angle_thresh):
    '''
    Creates a plasm that will capture openni data into a bag, using a dot pattern to sparsify views.
    
    @param bag_name: A filename for the bag, will write to this file.
    @param angle_thresh: The angle threshhold in radians to sparsify the views with.  
    '''

    plasm = ecto.Plasm()



    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   image_ci=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
                   depth_ci=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
                   )

    bagwriter = ecto.If('Bag Writer if R|T',
                        cell=ecto_ros.BagWriter(baggers=baggers, bag=bag_name)
                        )

    subs = dict(image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
                depth=ImageSub(topic_name='/camera/depth_registered/image', queue_size=0),
                image_ci=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
                depth_ci=CameraInfoSub(topic_name='/camera/depth_registered/camera_info', queue_size=0),
                )

    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs
                                 )
    keys = subs.keys()

    graph = [
                sync[:] >> bagwriter[keys],
            ]

    im2mat_rgb = ecto_ros.Image2Mat('rgb -> cv::Mat')
    camera_info = ecto_ros.CameraInfo2Cv('camera_info -> cv::Mat')
    poser = OpposingDotPoseEstimator(plasm,
                                     rows=5, cols=3,
                                     pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                     square_size=0.04, debug=True)

    bgr2rgb = imgproc.cvtColor('rgb -> bgr', flag=imgproc.Conversion.RGB2BGR)
    rgb2gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.Conversion.RGB2GRAY)
    delta_pose = capture.DeltaRT("delta R|T", angle_thresh=angle_thresh)
    display = highgui.imshow('Poses', name='Poses', waitKey=5, autoSize=True, triggers=dict(save=ord('s')))
    saver = ecto.If(cell=highgui.ImageSaver("saver", filename_format='ecto_image_%05d.jpg',
                                   start=1))

    graph += [sync['image'] >> im2mat_rgb[:],
              im2mat_rgb[:] >> (rgb2gray[:], bgr2rgb[:]),
              bgr2rgb[:] >> poser['color_image'],
              rgb2gray[:] >> poser['image'],
              poser['debug_image'] >> (display['input'], saver['image']),
              display['save'] >> saver['__test__'],
              sync['image_ci'] >> camera_info['camera_info'],
              camera_info['K'] >> poser['K'],
              poser['R', 'T', 'found'] >> delta_pose['R', 'T', 'found'],
              delta_pose['novel'] >> bagwriter['__test__'],
              ]
    plasm.connect(graph)
    return plasm


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

def create_capture_plasm_standalone(bag_name, angle_thresh,camera_file):
    from object_recognition.common.io.standalone.source import KinectReader
    from ecto_ros import Mat2Image, Cv2CameraInfo
    plasm = ecto.Plasm()
    
    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   image_ci=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
                   depth_ci=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
                   )

    bagwriter = ecto.If('Bag Writer if R|T',
                        cell=ecto_ros.BagWriter(baggers=baggers, bag=bag_name)
                        )
    
    kinect = KinectReader(plasm,camera_file)
    depthMsg = Mat2Image(frame_id='/camera_rgb_optical_frame')
    imageMsg = Mat2Image(frame_id='/camera_rgb_optical_frame')
    cameraInfoMsg = Cv2CameraInfo(frame_id='/camera_rgb_optical_frame')
    graph = [
                kinect['image'] >> imageMsg[:],
                kinect['depth'] >> depthMsg[:],
                kinect['K','D','image_size'] >> cameraInfoMsg['K','D', 'image_size'],
            ]
    graph += [
              imageMsg[:] >> bagwriter['image'],
              depthMsg[:] >> bagwriter['depth'],
              cameraInfoMsg[:] >> (bagwriter['image_ci'], bagwriter['depth_ci']),
              ]

    poser = OpposingDotPoseEstimator(plasm,
                                     rows=5, cols=3,
                                     pattern_type=calib.ASYMMETRIC_CIRCLES_GRID,
                                     square_size=0.04, debug=True)

    bgr2rgb = imgproc.cvtColor('rgb -> bgr', flag=imgproc.Conversion.RGB2BGR)
    rgb2gray = imgproc.cvtColor('rgb -> gray', flag=imgproc.Conversion.RGB2GRAY)
    delta_pose = capture.DeltaRT("delta R|T", angle_thresh=angle_thresh)
    display = highgui.imshow('Poses', name='Poses', waitKey=5, autoSize=True, triggers=dict(save=ord('s')))
    saver = ecto.If(cell=highgui.ImageSaver("saver", filename_format='ecto_image_%05d.jpg',
                                   start=1))

    graph += [kinect['image'] >> (rgb2gray[:], poser['color_image']),
              rgb2gray[:] >> poser['image'],
              poser['debug_image'] >> (display['input'], saver['image']),
              display['save'] >> saver['__test__'],
              kinect['K'] >> poser['K'],
              poser['R', 'T', 'found'] >> delta_pose['R', 'T', 'found'],
              delta_pose['novel'] >> bagwriter['__test__'],
              ]
    plasm.connect(graph)
    return plasm
