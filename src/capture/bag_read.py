#!/usr/bin/env python
import ecto
import ecto_ros, ecto_sensor_msgs
from ecto_opencv import highgui
import sys
import time

ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

def do_ecto():
    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   )
    
    bagreader = ecto_ros.BagReader('Bag Ripper',
                                    baggers=baggers,
                                    bag=sys.argv[1],
                                  )
    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()
    
    graph = [
                bagreader["image"] >> im2mat_rgb["image"],
                im2mat_rgb["image"] >> highgui.imshow("rgb show", name="rgb", waitKey=5)[:],
                bagreader["depth"] >> im2mat_depth["image"],
                im2mat_depth["image"] >> highgui.imshow("depth show", name="depth", waitKey= -1)[:]
            ]
    
    plasm = ecto.Plasm()
    plasm.connect(graph)
    
    sched = ecto.schedulers.Singlethreaded(plasm)
    return sched.execute()

    #sched = ecto.schedulers.Singlethreaded(plasm)
    #sched.execute()
if __name__ == "__main__":
    while True:
        do_ecto()
        time.sleep(0.1)