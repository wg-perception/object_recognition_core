
Readme
------

Readme, needs to be cleaned into rst... dumping here for now.
::   

   #sites (use chrome please):
   #http://localhost:5984/model_viewer/_design/viewer/index.html
   #*************** Setup up ROS ********************************
   #*************************************************************
   #Start up ros
   #terminal 1:
   roscore
   
   #Start up the openni camera -- plugin the camera, check!
   #terminal 2:
   roslaunch openni_camera openni_node.launch
   
   #*************** Setup up Capture Workspace*******************
   #*************************************************************
   #first capture an ORB template of your capture workspace. It
   #should be take from an planar frontal view, and the center of the image should
   #be filled by the plane. Press 's' to save an image. The result will
   #be placed in the directory given, e.g. my_textured_plane. Press 'q'
   #to quit the template capture program.
   #terminal 3:
   rosrun object_recognition_core orb_template.py -o my_textured_plane
   
   #try out tracking to see if you got a good template. Press 'q' to quit.
   rosrun object_recognition_core orb_track.py --track_directory my_textured_plane

   
   #*************** Capture Objects *****************************
   #*************************************************************
   #Once you are happy with the workspace tracking, its time to capure an object.
   #Place an object at the origin of the workspace. An run the capture program in preview mode.
   #Make sure the mask and pose are being picked up.
   rosrun object_recognition_core capture -i my_textured_plane --seg_z_min 0.01 -o silk.bag --preview

   #When satisified by the preview mode, run it for real.  The following will capture a bag of 60 views
   #where each view is normally distributed on the view sphere. The mask and pose displays should only refresh
   #when a novel view is captured.  The program will finish when 35 (-n) views are captured.
   #Press 'q' to quit early.
   rosrun object_recognition_core capture -i my_textured_plane --seg_z_min 0.01 -o silk.bag
   
   #Now time for upload. Give the object a name with, and useful tags seperated by a space, e.g. milk soy silk.
   rosrun object_recognition_core upload -i silk.bag -n 'Silk' milk soy silk --commit
   
   #*************** Train Objects *******************************
   #*************************************************************
   #repeat the steps above for the objects you would like to recognize.
   #Once you have captured and uploaded all of the data, it time to mesh and train object recognition.
         
   #Meshing objects can be done in a batch mode, assuming you are in the binary directory.
   rosrun object_recognition_core mesh_object --all --visualize --commit
   
   #Next objects should be trained. It may take some time between objects, this is normal.
   rosrun object_recognition_core training \
   -c `rospack find object_recognition_core`/bin/config_training.sample \
   --visualize
   
   #*************** Detect Objects ******************************
   #*************************************************************
   #now we're ready for detection. First launch rviz, it should be
   # subscribed to the right markers for recognition
   #results. /markers is used for the results, and it is a marker array.
   rosrun object_recognition_core detection \
   -c `rospack find object_recognition_core`/bin/config_detection.sample \
   #--visualize
