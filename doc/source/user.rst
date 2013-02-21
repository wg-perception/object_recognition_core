.. _userdoc:

User Documentation
==================

Setup Your Environment
----------------------

.. toggle_table::
    :arg1: Non-ROS
    :arg2: ROS


.. toggle:: Non-ROS

    Nothing.

.. toggle:: ROS

    Source your distribution ``setup.sh``.
    
    .. code-block:: bash
    
        source /opt/ros/*/setup.sh

If you built from source, you also need to run:

.. code-block:: bash

    source build/setup.sh

Or if the file is not present:

.. code-block:: bash

    source build/devel/setup.sh

This will add the built software to your ``PATH``, ``LD_LIBRARY_PATH`` and ``PYTHONPATH``.


General Usage
-------------

If you use ROS and want to try it out quickly without caring about the intricacies, please install as indicated above and follow:

.. toctree::  
  :maxdepth: 1

  ros_quickguide.rst

If you have a bit more time, we suggest you read about the two steps to object recognition: the capture to acquire data concerning an object and the recognition itself.

   * :ref:`capture <ork_capture:ork_capture>`
   * :ref:`recognition <object_recognition_core_user>`

Recognition Pipelines
---------------------

Several object recognition pipelines have been implemented for this framework. Their documentation is work in progress :) :

===========================================================================   =============================================   =============================================
Techniques                                                                    Types of object                                 Limitations                                  
===========================================================================   =============================================   =============================================
:ref:`LINE-MOD <ork_linemode:ork_linemod>`                                     * any rigid instance                            * does not work with partial occlusions
                                                                               * non-transparent                               * scales linearly with the number of objects
:ref:`tabletop <ork_tabletop:ork_tabletop>`                                    * rotationally symmetric                        * the object is assumed to be on a table
                                                                               * non-transparent                                 with no 3d rotation
                                                                               * also finds planar surfaces
:ref:`TOD <ork_tod:ork_tod>`                                                   * textured objects
:ref:`transparent objects<ork_transparent_objects:ork_transparent_obejcts>`    * transparent objects                           * Training has to be done on a painted 
                                                                                                                                 version of the object.                    
===========================================================================   =============================================   =============================================

There are also extra pipelines that can be used to help other tasks:

  * :ref:`reconstruction <ork_reconstruction:ork_reconstruction>`
