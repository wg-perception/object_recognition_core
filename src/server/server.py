#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import actionlib
from object_recognition_msgs.msg import *
from object_recognition_core.pipelines.plasm import create_detection_plasm
from object_recognition_core.utils.training_detection_args import read_arguments_detector
import ecto
import sys
import yaml

class RecognitionServer:
    def __init__(self,args):
        source_params, pipeline_params, sink_params, voter_params, args = read_arguments_detector()
        self.plasm = create_detection_plasm(source_params, pipeline_params, sink_params, voter_params)
        self.plasm.configure_all()
        print 'configured'
        self.sched = ecto.schedulers.Singlethreaded(self.plasm)
        
        #the results or the object recognition pipeline
        self.poses = None
        self.object_ids = None
        
        #subscribe to the output of the detection pipeline
        rospy.Subscriber("poses", PoseArray, self.callback_poses)
        rospy.Subscriber("object_ids", String, self.callback_object_ids)
        
        #actionlib stuff
        self.server = actionlib.SimpleActionServer('recognize_objects', ObjectRecognitionAction, self.execute, False)
        self.server.start()
    
    def callback_poses(self, data):
        self.poses = data
    
    def callback_object_ids(self, data):
        '''
        The data comming in here looks like yaml of the following form.
        data: {
            "object_ids":
            [
                "1599314c3968e9a3894e9a5a172579bd",
                "1599314c3968e9a3894e9a5a172579bd",
                "9b42ef52aa448de677fa6d2d288a4a67",
                "7bd55ac69a53277d544a10f438f54a4e"
            ]
        }
        
        ---
        '''
        obj = yaml.load(data.data) #parse the yaml into a dictionary
        self.object_ids = [] # create a list of object_ids.
	print obj
	if obj['object_ids'] is None:
	    print 'No objects found!'
	    return
        for x in obj['object_ids']:
            self.object_ids.append(str(x))

    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff here
        result = ObjectRecognitionResult()
        self.sched.execute(niter=1)
        #the pipeline should have published, wait for the results.
        while self.poses is None or self.object_ids is None:
            print 'waiting'
            rospy.sleep(0.1)
        result.header = self.poses.header
        result.poses = self.poses.poses
        result.object_ids = self.object_ids
        for pose in result.poses:
            result.confidence.append(1.0) # 50/50
        #we have a result!
        self.server.set_succeeded(result=result)
        
        #reset our instance variable for the next round
        self.poses = None
        self.object_ids = None

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)[1:]
    print 'rospy args stripped',args
    rospy.init_node('recognize_objects_server')
    server = RecognitionServer(args)
    rospy.spin()
