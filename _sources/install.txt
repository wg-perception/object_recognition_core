:orphan:

.. _install:

Installation
############

From the ROS repositories
*************************

Install the
``ros-*-object-recognition-core`` package from the official ROS repositories and the different pipelines (packages starting with ``ros-*-object-recognition-``).

Download & Build from Source
****************************

First, build your workspace:

.. code-block:: sh

   mkdir -p ork/src && cd ork/src


.. toggle_table::
   :arg1: Non-ROS
   :arg2: Fuerte
   :arg3: Groovy


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

.. toggle:: Fuerte

   First install catkin and source your ROS setup file:
   
   .. code-block:: sh
   
      sudo apt-get install libopenni-dev ros-fuerte-catkin ros-fuerte-ecto* ros-fuerte-opencv-candidate
      source /opt/ros/fuerte/setup.sh
   
   Install the catkin package from source as the package does not have the toplevel.cmake file:
   
   .. code-block:: bash
   
      git clone http://github.com/ros/catkin.git
      ln -s catkin/cmake/toplevel.cmake CMakeLists.txt
      cd ../ && git clone http://github.com/ros-infrastructure/catkin_pkg.git
      export PYTHONPATH=`pwd`/catkin_pkg/src:$PYTHONPATH
      cd src


.. toggle:: Groovy

   First source your ROS setup file:
   
   .. code-block:: sh
   
      sudo apt-get install libopenni-dev ros-groovy-catkin ros-groovy-ecto* ros-groovy-opencv-candidate ros-groovy-moveit-msgs
      source /opt/ros/groovy/setup.sh


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
   :arg2: Fuerte
   :arg3: Groovy

.. toggle:: Non-ROS

   Nothing for non-ROS.


.. toggle:: Fuerte

   .. code-block:: sh
   
      git clone http://github.com/wg-perception/object_recognition_msgs
      git clone http://github.com/wg-perception/object_recognition_ros && git checkout fuerte-devel

.. toggle:: Groovy

   .. code-block:: sh
   
      git clone http://github.com/wg-perception/object_recognition_msgs
      git clone http://github.com/wg-perception/object_recognition_ros 


and then build your code:


.. toggle_table::
   :arg1: Non-ROS
   :arg2: Fuerte
   :arg3: Groovy


.. toggle:: Non-ROS

   .. code-block:: sh
   
      cd ../ && mkdir build && cd build && cmake ../src && make


.. toggle:: Fuerte

   .. code-block:: sh
   
      cd ../ && mkdir build && cd build && cmake ../src && make


.. toggle:: Groovy

   .. code-block:: sh
   
      cd ../ && catkin_make



If you are a developer and have write access to the repositories, search and replace ``http://`` above and replace by
``git@github.com:``.


To maintain your code, each folder is each own ``git`` repository and you can pull/push from there.

Building the documentation
**************************

Before you can build the documentation (which you are reading right now), you need to have followed the installation
instructions and have already successfully called ``make`` in the ``build`` folder.

You then need some up to date packages:

.. code-block:: sh

   sudo pip install -U breathe sphinxcontrib-programoutput

From root, just type:

.. code-block:: sh

   cd build
   make doxygen
   make sphinx-doc

You will find the generated documentation under ``build/doc/html``.

Once the documentation is built, you can simply copy it (except for the ``.doctree`` folder) to the ``gh-pages`` branch
on GitHub.
