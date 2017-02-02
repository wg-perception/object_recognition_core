0.6.7 (2017-02-02)
------------------
* Merge pull request `#46 <https://github.com/wg-perception/object_recognition_core/issues/46>`_ from hris2003/master
  (fix `#45 <https://github.com/wg-perception/object_recognition_core/issues/45>`_)(ModelDocument)Get _id field directly from ViewIterator
* (fix)(ModelDocument)Get _id field directly from ViewIterator
* fix the rosinstall files
* Contributors: Ha Dang, Vincent Rabaud

0.6.6 (2016-04-24)
------------------
* add missing dependencies
* fix doc according to https://github.com/wg-perception/reconstruction/issues/6
* simplify OpenCV3 compatibility
* add citation info in the docs
  fixes `#34 <https://github.com/wg-perception/object_recognition_core/issues/34>`_
* Contributors: Vincent Rabaud

0.6.5 (2015-02-12)
------------------
* Properly install test macros
  This fixes `#33 <https://github.com/wg-perception/object_recognition_core/issues/33>`_
* Don't throw when database is empty
  There is no need to throw here.
  The function load_fields_and_attachments works quite nicely
  for an empty database. The only thing it has to do is return...
  This makes it possible to use the rviz plugin OrkObject without
  a valid database (This obviously doesn't show meshes or the name then,
  but it's still useful as it prints confidence values and the object's key.
* Contributors: Michael Görner, Vincent Rabaud

0.6.4 (2015-01-20)
------------------
* fix training pipeline creation
* allow unicode strings
  this should fix https://github.com/wg-perception/reconstruction/issues/1
* Update installation instruction to add ork_visualization
* add the tutorials on the front page
* get code to work with OpenCV3
* OpenCV 3.0 adaptation
* Contributors: Ha Dang, Vincent Rabaud, edgarriba, nlyubova

0.6.3 (2014-09-06)
------------------
* Merge pull request `#27 <https://github.com/wg-perception/object_recognition_core/issues/27>`_ from cottsay/master
  Always include ecto.hpp first
* Always include ecto.hpp first
  ecto.hpp includes Python.hpp, which must always come before system includes
* adapt docs for ROS distribution
* improve install instructions
  This fixes `#25 <https://github.com/wg-perception/object_recognition_core/issues/25>`_ by only taking non-redundant information
* Contributors: Scott K Logan, Vincent Rabaud

0.6.2 (2014-05-13)
------------------
* add catkin-sphinx to the docs
* Fixes https://github.com/wg-perception/object_recognition_ros/issues/15
* Fix detection: error: unrecognized arguments:
* Merge pull request `#19 <https://github.com/wg-perception/object_recognition_core/issues/19>`_ from bit-pirate/master
  adds ork_ros to ork.rosinstall
* Contributors: Sammy Pfeiffer, Vincent Rabaud, hdang

0.6.1 (2014-04-13)
------------------
* get code to compile on Indigo
* allow flexibility in the inputs for the cloud
* improve the viewer for the first load
* fix docs
* replace the mesh viewer by jsc3d
  jsc3d is more flexible, handles obj/stl and has a better zoom.
  The different possible meshes are also handled better
* fix typo
* install the web UI no matter what
* install the db_scripts
* Contributors: Vincent Rabaud

0.6.0 (2013-12-18  21:12:06 +0100)
----------------------------------
- fix docs
- better mesh handling
- drop Fuerte support
