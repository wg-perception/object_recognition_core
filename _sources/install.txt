:orphan:

.. _install:

To use ORK, you will need a compiled ORK and couchDB (for object management). Following instructions in this page will help you to:
 - Install ORK either from precompiled ROS packages or from source code
 - Install couchDB and setup the database for ORK
 - Compile the documentation so that you can consult it locally (useful when no internet connection is around)

Installation
############

From the ROS repositories
*************************

Install the ``ros-*-object-recognition-core`` package from the official ROS repositories and the different pipelines (packages starting with ``ros-*-object-recognition-``).

Download & Build from Source
****************************

First, build your workspace:

.. code-block:: sh

   mkdir -p ork/src && cd ork/src


.. toggle_table::
   :arg1: Non-ROS
   :arg2: ROS


.. toggle:: Non-ROS

   If you want to install from source without ROS, you need to have common dependencies (OpenCV, PCL) on your path. You also need to execute the following:
   
   
   .. code-block:: bash
   
      git clone http://github.com/ros/catkin.git
      ln -s catkin/cmake/toplevel.cmake CMakeLists.txt
      cd ../ && git clone http://github.com/ros-infrastructure/catkin_pkg.git
      export PYTHONPATH=`pwd`/catkin_pkg/src:$PYTHONPATH
      cd src
      
   
   ``catkin`` is a set of CMake macros that simplify build and maintenance.
   
   Then install the ``ecto`` modules:
   
   .. code-block:: sh
   
      git clone http://github.com/plasmodic/ecto
      git clone http://github.com/plasmodic/ecto_image_pipeline
      git clone http://github.com/plasmodic/ecto_openni
      git clone http://github.com/plasmodic/ecto_opencv
      git clone http://github.com/plasmodic/ecto_pcl
      git clone http://github.com/plasmodic/ecto_ros
      git clone http://github.com/wg-perception/opencv_candidate


.. toggle:: ROS

   First source your ROS setup file (adapt ``DISTRO`` to the ROS distribution you are using):
   
   .. code-block:: sh
   
      export DISTRO=indigo
      sudo apt-get install libopenni-dev ros-${DISTRO}-catkin ros-${DISTRO}-ecto* ros-${DISTRO}-opencv-candidate ros-${DISTRO}-moveit-msgs
      source /opt/ros/${DISTRO}/setup.sh


Then install any pipeline you need:

.. code-block:: sh

   git clone http://github.com/wg-perception/object_recognition_core
   git clone http://github.com/wg-perception/capture
   git clone http://github.com/wg-perception/reconstruction
   git clone http://github.com/wg-perception/linemod
   git clone http://github.com/wg-perception/ork_renderer
   git clone http://github.com/wg-perception/tabletop
   git clone http://github.com/wg-perception/tod
   git clone http://github.com/wg-perception/transparent_objects

any ROS stuff:

.. toggle_table::
   :arg1: Non-ROS
   :arg3: ROS

.. toggle:: Non-ROS

   Nothing for non-ROS.


.. toggle:: ROS

   .. code-block:: sh
   
      git clone http://github.com/wg-perception/object_recognition_msgs
      git clone http://github.com/wg-perception/object_recognition_ros
      git clone http://github.com/wg-perception/object_recognition_ros_visualization


and then build your code:


.. toggle_table::
   :arg1: Non-ROS
   :arg2: ROS


.. toggle:: Non-ROS

   .. code-block:: sh
   
      cd ../ && mkdir build && cd build && cmake ../src && make


.. toggle:: ROS

   .. code-block:: sh
   
      cd ../ && catkin_make



If you are a developer and have write access to the repositories, search and replace ``http://`` above and replace by ``git@github.com:``.


To maintain your code, each folder is each own ``git`` repository and you can pull/push from there.

rosinstall file
***************

Under ROS, you can alternatively use that `rosinstall <http://www.ros.org/wiki/rosinstall>`_ file :download:`ork.rosinstall.indigo.jade`
on Indigo/Jade or :download:`ork.rosinstall.kinetic.plus` on Kinetic and above.

To use on Kinetic, do the following:

.. code-block:: bash

   mkdir ws && cd ws
   wstool init src https://raw.github.com/wg-perception/object_recognition_core/master/doc/source/ork.rosinstall.kinetic.plus
   cd src && wstool update -j8
   cd .. && rosdep install --from-paths src -i -y
   catkin_make
   source devel/setup.bash

Configuring the database
########################

For that, just refer to the :ref:`DB Page <object_recognition_core_db>`.


Building the documentation
##########################

Before you can build the documentation (which you are reading right now), you need to have followed the installation instructions and have already successfully called ``make`` in the ``build`` folder.

You then need some up to date packages:

.. code-block:: sh

   sudo pip install -U breathe catkin-sphinx sphinxcontrib-programoutput

From root, just type:

.. code-block:: sh

   cd build
   make doxygen
   make sphinx-doc

You will find the generated documentation under ``build/doc/html``.

Once the documentation is built, you can simply copy it (except for the ``.doctree`` folder) to the ``gh-pages`` branch on GitHub.
