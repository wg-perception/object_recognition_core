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
