Data Capture
============

.. contents::

Object capture tools.

setup
-----
Capture is view based, and requires a fiducial that is rigidly attached to
the object being observed.  This enables relatively accurate view point pose estimation,
a consistent object coordinate frame, and simple object/background segmentation.
The setup assumes that you have an RGB Depth device, such as the Kinect.

.. _capture_board:

The fiducial currently used for object capture is available for
download here: :download:`capture_board_big_5x3.svg`

.. figure:: capture_board_big_5x3.svg.png

  The default capture board uses circle pattern based fiducial markers,
  one black on white, the other inverted, so that two may be detected in
  the scene and allow for pose estimation in the presence of occlusion
  of one of the markers.

Get a full size printing of the above fiducial marker and mount it to flat surface,
possibly on a lazy susan. http://en.wikipedia.org/wiki/Lazy_Susan

.. highlight:: ectosh

capture
-------

``capture`` is the entry point for using the our object capture system.
The capture program will estimate a pose per view, along with a depth based mask.
This will result in a ROS bag of data that has the following topics::

   types:       geometry_msgs/PoseStamped [d3812c3cbc69362b77dc0b19b345f8f5]
                sensor_msgs/CameraInfo    [c9a58c1b0b154e0e6da7578cb991d214]
                sensor_msgs/Image         [060021388200f6f0f447d0fcd9c64743]
   topics:      /camera/depth/camera_info   72 msgs    : sensor_msgs/CameraInfo   
                /camera/depth/image         72 msgs    : sensor_msgs/Image        
                /camera/mask                72 msgs    : sensor_msgs/Image        
                /camera/pose                72 msgs    : geometry_msgs/PoseStamped
                /camera/rgb/camera_info     72 msgs    : sensor_msgs/CameraInfo   
                /camera/rgb/image_color     72 msgs    : sensor_msgs/Image


ROS OpenNI Startup
^^^^^^^^^^^^^^^^^^
The ROS drivers for openni are used to capture a view sparse bag of data.
Please make sure you are on ``electric`` or the drivers.

See http://www.ros.org/wiki/electric/Installation/Ubuntu and http://ros.org/wiki/openni
for detailed instructions.

Before you start capturing data, please start up the ROS OpenNI driver::

   % roslaunch openni_launch openni.launch

Make sure that the depth_registration mode for the driver is set to true::

   % rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True

It may be preferable to use the SXGA (roughly 1 megapixel) mode of your openni device::

   % rosrun dynamic_reconfigure dynparam set /camera/driver image_mode 1

use
^^^

To use ``capture`` you should place your object in the center of the fiducial board, and keep it in the same location
for the entirety of the capture session. Slowly turn the fiducial board, and the program should capture views that are
evenly distributed in a view pose sphere.

A typical command line session might look like::

   % apps/capture --seg_z_min 0.01 -o data/object_recognition_capture/green_mug_03.bag
   [ INFO] [1317675185.049932343]: Initialied ros. node_name: /object_capture
   [ INFO] [1317675185.071477755]: Subscribed to topic:/camera/rgb/camera_info with queue size of 0
   [ INFO] [1317675185.072543173]: Subscribed to topic:/camera/depth_registered/camera_info with queue size of 0
   [ INFO] [1317675185.073481893]: Subscribed to topic:/camera/depth_registered/image with queue size of 0
   [ INFO] [1317675185.074486101]: Subscribed to topic:/camera/rgb/image_color with queue size of 0
   Opening bag: data/object_recognition_capture/green_mug_03.bag
   Satisfied, total observations: 72

You should see an popup image similar to the following:

.. figure:: capture.gif

  A sample sequence of view captured using an opposing dot pattern fudicial marker.

command line interface
^^^^^^^^^^^^^^^^^^^^^^
.. program-output:: apps/capture --help
   :in_srcdir:
   :until: Scheduler Options:


multiple capture sessions
^^^^^^^^^^^^^^^^^^^^^^^^^
If you decided to take multiple bags of an object, from different view points,
please concatenate the bags before upload. However, if you moved the object on the board, then you should consider
these bags as seperate "sessions" of the same object.

There is a convenience script for this called ``concat.py``

.. program-output:: apps/bagscripts/concat.py --help
   :in_srcdir:

upload
------
Once you have captured a bag of views, you will want to upload the bag to the database.  This upload will contain all
of the views in the bag, plus some meta information about the object. It assumed that each bag has one object,
and this object has a consistent coordinate frame throughout the bag.

use
^^^
A typical command line session will look like::

   % apps/upload -a 'Ethan Rublee' -e 'erublee@willowgarage.com' -i silk_highres.bag -n 'silk' -d 'A carton of Silk brand soy milk.' --commit milk, soy, kitchen, tod
   Uploaded session with id: 4ad9f2d3db57bbd414e5e987773490a0

If you leave off the ``--commit`` the script will run without actually committing anything to
the database.

Now that the bag is uploaded, into the database, you can see it in the db by browsing to:

* http://localhost:5984/_utils/database.html?object_recognition/_design/objects/_view/by_object_name

command line interface
^^^^^^^^^^^^^^^^^^^^^^
.. program-output:: apps/upload --help
   :in_srcdir:
   :until: Scheduler Options:

Willow users
^^^^^^^^^^^^
Some pre-acquired bags exist internally for now, just rsync them::

   % rsync -vPa /wg/wgss0_shelf1/object_recognition_capture ./
