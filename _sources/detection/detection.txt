:orphan:

.. _detection:

Detection
#########
.. highlight:: ectosh

.. contents::

Using the different trained objects, we can now detect them.

Use
***

.. toggle_table::
    :arg1: Non-ROS
    :arg2: ROS

.. toggle:: Non-ROS

   Just run the detection.py script in /apps. This will run continuously on the input image/point cloud.

   .. code-block:: sh

      ./apps/detection -c detection.ork

   The server requires a configuration file through the ``-c`` option.

.. toggle:: ROS

   If you want continuous detection, you can just run the detection script:

   .. code-block:: sh

      rosrun object_recognition_core detection -c `rospack find object_recognition_tod`/conf/detection.ros.ork

   Then again, there is also an actionlib server as detailed on :ref:`actionlib server <orkros:actionlib>`:

   .. code-block:: sh

      rosrun object_recognition_ros server -c `rospack find object_recognition_tod`/conf/detection.ros.ork

   This will start a server with a given configuration file.
   If you want to test the server, just execute the client once:

   .. code-block:: sh

      rosrun object_recognition_ros client

   You can also use roslaunch if you want traditional actionlib support. There is a ``config_file`` argument
   that can help you choose different pipelines:

   .. code-block:: sh

      roslaunch object_recognition_ros server.robot.launch

A typical command line session might look like::

   % apps/detection -c `rospack find object_recognition_tod`/conf/detection.ros.ork
   [ INFO] [1317692023.717854617]: Initialized ros. node_name: /ecto_node_1317692023710501315
   Threadpool executing [unlimited] ticks in 5 threads.
   [ INFO] [1317692024.254588151]: Subscribed to topic:/camera/rgb/camera_info with queue size of 0
   [ INFO] [1317692024.255467268]: Subscribed to topic:/camera/depth_registered/camera_info with queue size of 0
   [ INFO] [1317692024.256186358]: Subscribed to topic:/camera/depth_registered/image with queue size of 0
   [ INFO] [1317692024.256863212]: Subscribed to topic:/camera/rgb/image_color with queue size of 0
   model_id: e2449bdc43fd6d9dd646fcbcd012daaa
   span: 0.433393 meters
   1
   ***Starting object: 0
   * starting RANSAC
    added : 1
    added : 0
   * n inliers: 1824
   [-0.056509789, 0.99800211, 0.028263446;
     0.94346958, 0.062639669, -0.32548648;
     -0.32660651, 0.0082725696, -0.94512439]
   [-0.32655218; 0.03684178; 0.85040951]
   ********************* found 1poses
   [ INFO] [1317692117.187226953]: publishing to topic:/object_ids
   [ INFO] [1317692117.188155476]: publishing to topic:/poses


Command Line Interface
**********************
.. program-output:: ../../../apps/detection --help
   :in_srcdir:

Configuration File
******************

The configuration file is where you define your graph and with the current ORK, you can choose any of the following sources:

.. program-output:: python -c "from object_recognition_core.utils.doc import config_yaml_for_ecto_cells; print '\n'.join(config_yaml_for_ecto_cells('source'))"
   :shell:

any of the following sinks:

.. program-output:: python -c "from object_recognition_core.utils.doc import config_yaml_for_ecto_cells; print '\n'.join(config_yaml_for_ecto_cells('sink'))"
   :shell:

or the following pipelines:

.. program-output:: python -c "from object_recognition_core.utils.doc import config_yaml_for_ecto_cells; print '\n'.join(config_yaml_for_ecto_cells('detection_pipeline'))"
   :shell:

More of any of those can be added by the user obviously
