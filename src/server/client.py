#!/usr/bin/env python
import rospy
import actionlib
from object_recognition_msgs.msg import *

def on_result(status, result):
  print result

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)[1:]
    print 'rospy args stripped',args
    if args != ['--help']:
        rospy.init_node('recognition_client')
        client = actionlib.SimpleActionClient('recognize_objects', ObjectRecognitionAction)
        client.wait_for_server()
    
        start = rospy.Time.now() # for checking the round trip time.
        client.send_goal(ObjectRecognitionGoal(),done_cb=on_result)
        client.wait_for_result() # wait indefinitely for a result
    
        #print out the round trip time.
        print "Time for 1 detection:", (rospy.Time.now() - start).to_sec()
