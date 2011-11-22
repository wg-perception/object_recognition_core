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
