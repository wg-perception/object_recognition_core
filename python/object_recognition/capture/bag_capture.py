import ecto
from ecto_opencv import highgui
from object_recognition.common.io.source import Source, SourceTypes
import ecto_sensor_msgs,ecto_ros
ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

def create_openni_bag_capture_plasm(bag_name, source_type=SourceTypes.ros_kinect):
    '''
    Creates a plasm that will capture RGBD data on a key press.
    :returns: A plasm that when executed will subscribe to an openni and save frames on the keypress 's'
    '''
    graph = []
    source = Source.create_source(source_type)
    display = highgui.imshow(name='image', triggers=dict(save=ord('s')))
    display_depth = highgui.imshow(name='depth')

    graph += [source['image'] >> display['image'],
              source['depth'] >> display_depth['image']
              ]

    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
               depth=ImageBagger(topic_name='/camera/depth/image'),
               image_ci=CameraInfoBagger(topic_name='/camera/rgb/camera_info'),
               depth_ci=CameraInfoBagger(topic_name='/camera/depth/camera_info'),
               )
    bagwriter = ecto.If('Bag Writer if R|T',
                        cell=ecto_ros.BagWriter(baggers=baggers, bag=bag_name)
                        )
    bag_keys = ('image', 'depth', 'image_ci', 'depth_ci')
    source_keys = ('image_message', 'depth_message', 'image_info_message', 'depth_info_message')

    graph += [source[source_keys] >> bagwriter[bag_keys],
              ]
    graph += [
          display['save'] >> bagwriter['__test__'],
          ]
    plasm = ecto.Plasm()
    plasm.connect(graph)
    return plasm
