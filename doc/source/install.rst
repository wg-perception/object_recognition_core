.. _install:

Installation
============

From the ROS repositories
-------------------------

Install the ``ros-*-object-recognition-core`` package from the official ROS repositories and the different pipelines (packages starting with ``ros-*-object-recognition-``).

Download & Build from Source
----------------------------

.. toggle_table::
    :arg1: Non-ROS
    :arg2: Fuerte
    :arg3: Groovy


.. toggle:: Non-ROS

    If you want to install from source without ROS, you need to have common dependencies (OpenCV, PCL) on your path. You also need to execute the following:

    .. code-block:: sh

        sudo pip install rosinstall

    Write a .rosinstall file containing:

    the core:

    .. literalinclude:: ../../install/rosinstall_main
        :language: yaml


.. toggle:: Fuerte

    First source your ROS setup file:

    .. code-block:: sh

        source /opt/ros/fuerte/setup.sh

    Install rosinstall if you don't already have it:

    .. code-block:: sh

        sudo apt-get install python-rosinstall

    Write a .rosinstall file containing:

    the core:

    .. literalinclude:: ../../install/rosinstall_main_fuerte
        :language: yaml

.. toggle:: Groovy

    First source your ROS setup file:

    .. code-block:: sh

        source /opt/ros/groovy/setup.sh

    Install rosinstall if you don't already have it:

    .. code-block:: sh

        sudo apt-get install python-rosinstall

    Write a .rosinstall file containing:

    the core:

    .. literalinclude:: ../../install/rosinstall_main
        :language: yaml

the pipelines:

.. literalinclude:: ../../install/rosinstall_pipelines
    :language: yaml

ROS stuff:

.. toggle_table::
    :arg1: Non-ROS
    :arg2: Fuerte
    :arg3: Groovy

.. toggle:: Non-ROS

    Nothing for non-ROS.


.. toggle:: Fuerte

    .. literalinclude:: ../../install/rosinstall_ros_fuerte
        :language: yaml


.. toggle:: Groovy

    .. literalinclude:: ../../install/rosinstall_ros
       :language: yaml


You'll notice that one of the packages is ``catkin`` http://www.ros.org/wiki/catkin. It is a set of CMake macros that simplify build and maintenance.
If you are a developer and have write access to the repositories, search and replace ``https://`` above and replace by ``git@github.com:``.

Then simply run:

.. code-block:: sh

    rosinstall . .rosinstall


Set the main ``CMakeLists.txt`` file:

.. toggle_table::
    :arg1: Non-ROS
    :arg2: Fuerte
    :arg3: Groovy

.. toggle:: Non-ROS

    .. code-block:: sh

        ln -s catkin/cmake/toplevel.cmake CMakeLists.txt



.. toggle:: Fuerte

    .. code-block:: sh

        ln -s catkin/toplevel.cmake CMakeLists.txt



.. toggle:: Groovy

    .. code-block:: sh

        ln -s catkin/cmake/toplevel.cmake CMakeLists.txt


The different components should then be copied over.

From here on, it's a normal CMake build:

.. code-block:: sh

    mkdir build
    cd build
    cmake ..
    make

To maintain your code, each folder is each own ``git`` repository and you can pull/push from there. Little convenience: if you want to update everything:

.. code-block:: sh

    source build/buildspace/setup.sh
    catkin_ws pull
    
Building the documentation
--------------------------

Before you can build the documentation (which you are reading right now),
you need to have followed the installation instructions and have already
successfully called ``make`` in the ``build`` folder.

You will need some up to date packages:
::

  sudo pip install -U breathe sphinxcontrib-programoutput

And you will need to add the object_recognition_doc package to your workspace:

  https://github.com/wg-perception/object_recognition_doc

or add to your rosinstall:

.. code-block:: yaml

  - git: {local-name: object_recognition_doc, uri: 'https://github.com/wg-perception/object_recognition_doc.git'}

From the root, just type:
::

  cd build
  make doxygen
  make doc
  
You will find the generated documentation under ``build/doc/html``.

If you want to upload the documentation to the webserver, it's a bit harder because of intersphinx (you will alsoneed developer access to do that).

First, build the docs making sure that ``ecto_module_root`` is set to the source version in ``ecto/doc/kitchen/doc_config.py``. Then rsync
::


  rsync -vrz --delete build/doc/html/ecto/ ecto.willowgarage.com:/var/www/
  rsync -vrz --delete build/doc/html/object_recognition_doc/ ecto.willowgarage.com:/var/www/recognition
  rsync -vrz --delete build/doc/html/ecto_* ecto.willowgarage.com:/var/www/

Then, build the docs again setting ``ecto_module_root`` to the release version. Then rsync again
::

  rsync -vrz --delete build/doc/html/ecto/ ecto.willowgarage.com:/var/www/
  rsync -vrz --delete build/doc/html/object_recognition_doc/ ecto.willowgarage.com:/var/www/recognition
  rsync -vrz --delete build/doc/html/ecto_* ecto.willowgarage.com:/var/www/



