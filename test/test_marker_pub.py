#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker

'''
markers: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1317880841
        nsecs: 99929979
      frame_id: /camera_rgb_optical_frame
    ns: ''
    id: 0
    type: 10
    action: 0
    pose: 
      position: 
        x: -0.110998123884
        y: -0.00887924432755
        z: 0.657616496086
      orientation: 
        x: 0.917710006237
        y: -0.289691776037
        z: 0.103748500347
        w: 0.251243621111
    scale: 
      x: 1.0
      y: 1.0
      z: 1.0
    color: 
      r: 1.0
      g: 1.0
      b: 0.0
      a: 0.0
    lifetime: 
      secs: 5
      nsecs: 0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: http://localhost:5984/object_recognition/cd4c27a2dd307cda79b49ee31494ba08/mesh.stl
    mesh_use_embedded_materials: True
'''
def talker():
    pub = rospy.Publisher('marker_chatter', Marker)
    rospy.init_node('marker_talker')
    while not rospy.is_shutdown():
        m = Marker()
        m.header.frame_id =  '/camera_rgb_optical_frame'
        m.type = 10
        m.id = 2
        m.action = 0
        m.scale.x = 1
        m.scale.y = 1
        m.scale.z = 1
        m.color.r = 0.3
        m.color.g = 1
        m.color.b = 0.2
        m.color.a = 0.7
        m.lifetime.secs = 1
        m.frame_locked = False
        m.mesh_resource= 'http://localhost:5984/object_recognition/cd4c27a2dd307cda79b49ee31494ba08/mesh.stl'
        m.mesh_use_embedded_materials= False
        pub.publish(m)
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
