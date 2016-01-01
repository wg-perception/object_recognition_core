:orphan:

.. _quickguide:

Quick Guide
###########

Instructions are slightly different on whether you use ROS or not. For ROS, instructions will work whether you build ``ROS`` from source or not.

.. toggle_table::
   :arg1: From Source
   :arg2: From ROS packages

Install
*******

Install ``ORK`` as detailed on the :ref:`Install <install>` page.

If you want to visualize data in the database, you also need to run (be sure to have installed couchapp sudo pip install -U couchapp):

.. toggle:: From Source

   .. code-block:: bash

      cd object_recognition_core/web_ui && ./push.sh

.. toggle:: From ROS packages

   .. code-block:: bash

      rosrun object_recognition_core push.sh

Setup Your Environment
**********************

You need to setup your environment:

.. toggle:: From Source

   .. code-block:: bash

      source devel/setup.sh

.. toggle:: From ROS packages

   .. code-block:: bash

      source /opt/ros/DISTRO_NAME/setup.sh

This will add all the necessary packages to your ``PATH``, ``LD_LIBRARY_PATH`` and ``PYTHONPATH``.

.. toggle:: From ROS packages

   .. rubric:: Setup ROS

   Terminal 1:

   .. code-block:: sh

      roscore

   Terminal 2:

   .. code-block:: sh

      roslaunch openni_launch openni.launch

Setup the capture workspace
***************************

First capture an ORB template of your capture workspace. It  should be take from an planar frontal view, and the center of the image should be filled by the plane. Press 's' to save an image. The result will be placed in the directory given, e.g. my_textured_plane. Press 'q' to quit the template capture program.

.. toggle:: From Source

   .. code-block:: sh

      ./object_recognition_capture/apps/orb_template -o my_textured_plane

.. toggle:: From ROS packages

   Terminal 3:

   .. code-block:: sh

      rosrun object_recognition_capture orb_template -o my_textured_plane

   Try out tracking to see if you got a good template. Press 'q' to quit.

   .. code-block:: sh

      rosrun object_recognition_capture orb_track --track_directory my_textured_plane

   Uuse the SXGA (roughly 1 megapixel) mode of your openni device if possible.

   .. code-block:: sh

      rosrun dynamic_reconfigure dynparam set /camera/driver image_mode 1
      rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True

Capture objects
***************

Once you are happy with the workspace tracking, its time to capure an object. Place an object at the origin of the workspace. An run the capture program in preview mode. Make sure the mask and pose are being picked up.

.. toggle:: From Source

   .. code-block:: sh

      ./object_recognition_capture/apps/capture -i my_textured_plane --seg_z_min 0.01 -o silk.bag --preview

.. toggle:: From ROS packages

   .. code-block:: sh

      rosrun object_recognition_capture capture -i my_textured_plane --seg_z_min 0.01 -o silk.bag --preview

When satisified by the preview mode, run it for real.  The following will capture a bag of 60 views where each view is normally distributed on the view sphere. The mask and pose displays should only refresh when a novel view is captured. The program will finish when 35 (-n) views are captured. Press 'q' to quit early.

.. toggle:: From Source

   .. code-block:: sh

      ./object_recognition_capture/apps/capture -i my_textured_plane --seg_z_min 0.01 -o silk.bag

.. toggle:: From ROS packages

   .. code-block:: sh

      rosrun object_recognition_capture capture -i my_textured_plane --seg_z_min 0.01 -o silk.bag

Now time for upload. Make sure you install couch db on your machine. Give the object a name and useful tags seperated by a space, e.g. milk soy silk.

.. toggle:: From Source

   .. code-block:: sh

      ./object_recognition_capture/apps/upload -i silk.bag -n 'Silk' milk soy silk --commit

.. toggle:: From ROS packages

   .. code-block:: sh

      rosrun object_recognition_capture upload -i silk.bag -n 'Silk' milk soy silk --commit

Train objects
*************

Repeat the steps above for the objects you would like to recognize. Once you have captured and uploaded all of the data, it time to mesh and train object recognition.

Meshing objects can be done in a batch mode as follows:


.. toggle:: From Source

   .. code-block:: sh

      ./object_recognition_reconstruction/apps/mesh_object --all --visualize --commit

.. toggle:: From ROS packages

   .. code-block:: sh

    rosrun object_recognition_reconstruction mesh_object --all --visualize --commit

The currently stored models are on http://localhost:5984/or_web_ui/_design/viewer/meshes.html

Next objects should be trained. It may take some time between objects, this is normal. Also, this quickguide assumes that you are using :ref:`TOD <orktod:tod>` which only works for textured objects. Please refer to the documentation of other methods.

.. toggle:: From Source

   .. code-block:: sh

      ./object_recognition_core/apps/training \
      -c object_recognition_tod/conf/training.ork \
      --visualize

.. toggle:: From ROS packages

   .. code-block:: sh

      rosrun object_recognition_core training \
      -c `rospack find object_recognition_tod`/conf/training.ork \
      --visualize

Detect objects
**************

Now we're ready for detection. First launch rviz, it should be subscribed to the right markers for recognition results. /markers is used for the results, and it is a marker array.

.. toggle:: From Source

   .. code-block:: sh

      ./rosrun object_recognition_core/apps/detection \
      -c object_recognition_tod/conf/detection.ork \
      --visualize

.. toggle:: From ROS packages

   .. code-block:: sh

      rosrun object_recognition_core detection \
      -c `rospack find object_recognition_tod`/conf/detection.ros.ork \
      --visualize
