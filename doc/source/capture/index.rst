Model Capture
=============

Object capture tools.

Capture setup
-------------
The model capture is based on a view

.. _capture_board:

Download: :download:`capture_board_big_5x3.svg`

.. image:: capture_board_big_5x3.svg
  :alt: The fiducial board used for pose estimation during capture.

  The default capture board uses circle pattern based fiducial markers,
  one black on white, the other inverted, so that two may be detected in
  the scene and allow for pose estimation even in the presence of occlusion.

Get a full size printing of the above fiducial marker and mount it to flat surface,
possibly on a lazy susan. http://en.wikipedia.org/wiki/Lazy_Susan

.. todo:: Support many sizes/types of fiducial markers through parameters.

Capture a Bag
-------------

The ROS drivers for openni are used to capture a view sparse bag of data.
Please make sure you are on ``electric`` or the drivers.

See http://www.ros.org/wiki/electric/Installation/Ubuntu for detailed instructions.

:

