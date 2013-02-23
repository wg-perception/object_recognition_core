Object Recognition Kitchen
##########################

.. highlight:: ectosh

The Object Recognition Kitchen (``ORK``) is a project started at Willow Garage for object recognition.

There is currently no unique method to perform object recognition. Objects can be textured, non textured, transparent, articulated, etc.
For this reason, the Object Recognition Kitchen was designed to easily develop and run simultaneously several object recognition techniques.
In short, ORK takes care of all the non-vision aspects of the problem for you (database management, inputs/outputs handling, robot/ROS integration ...) and
eases reuse of code.

``ORK`` is built on top of `ecto <http://ecto.willowgarage.com>`_ which is a lightweight hybrid C++/Python framework for organizing computations as directed acyclic graphs.


Quickguide
**********

We know you can't wait; if you don't care about the intricacies and want to have a
quick overview, follow this:

.. toctree::
  :maxdepth: 1

  quickguide.rst

General Usage
*************

Ok, now that you have a bit more time, we suggest you learn about the two steps to object recognition:

  * the :ref:`capture <orkcapture:ork_capture>` to acquire data concerning an object

And also:

.. toctree::
  :maxdepth: 1

  the recognition <index_user>

ROS integration
***************

The recognition kitchen was built in a ROS agnostic way, but ROS components were
also developed for integration with the ROS ecosystem (e.g. publishers, subscribers,
actionlib server, RViz plugin ...).

  * :ref:`ROS integration <orkros:ros>`

Recognition Pipelines
*********************

Several object recognition pipelines have been implemented for this framework. Their documentation is work in progress :) :


======================================================================        ==============================                  ===============================================
Techniques                                                                    Types of object                                 Limitations                                  
======================================================================        ==============================                  ===============================================
:ref:`LINE-MOD <orklinemod:line_mod>`                                          * any rigid instance                            * does not work with partial occlusions
                                                                               * non-transparent                               * scales linearly with the number of objects
:ref:`tabletop <orktabletop:tabletop>`                                         * rotationally symmetric                        * the object is assumed to be on a table
                                                                               * non-transparent                                 with no 3d rotation
                                                                               * also finds planar surfaces
:ref:`TOD <orktod:tod>`                                                        * textured objects
:ref:`transparent objects<orktransparentobjects:transparent_objects>`          * transparent objects                           * Training has to be done on a painted 
                                                                                                                                 version of the object.                    
======================================================================        ==============================                  ===============================================

Tools
*****

There are several toolsalso extra pipelines that can be used to help other tasks:

  * :ref:`reconstruction <orkreconstruction:reconstruction>`

Developers' corner
##################

You like ``ORK`` ? Well you can add any pipeline or database to it. It is fairly simple (100-150 lines to wrap any
technique out there):

.. toctree::
  :maxdepth: 1

  developing with ORK <index_developer>

Contacts
########

For bug reports and comments, please use the github infrastructure at https://github.com/wg-perception/
