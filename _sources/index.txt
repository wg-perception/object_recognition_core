.. _index:

Object Recognition Kitchen
##########################

.. highlight:: ectosh

The Object Recognition Kitchen (``ORK``) is a project started at Willow Garage for object recognition.

There is currently no unique method to perform object recognition. Objects can be textured, non textured, transparent, articulated, etc. For this reason, the Object Recognition Kitchen was designed to easily develop and run simultaneously several object recognition techniques. In short, ORK takes care of all the non-vision aspects of the problem for you (database management, inputs/outputs handling, robot/ROS integration ...) and eases reuse of code.

``ORK`` is built on top of `ecto <http://plasmodic.github.com/ecto>`_ which is a lightweight hybrid C++/Python framework for organizing computations as directed acyclic graphs.

.. rubric:: Install

Well, all good things must start so check out the :ref:`Install <install>`.

.. rubric:: Quickguide

We know you can't wait; if you don't care about the intricacies and want to have a quick overview, follow the :ref:`Quick Guide <quickguide>`

.. rubric:: Tutorials

Ok, now that you know a little, you can follow the :ref:`Tutorials <orktutorials:object_recognition_tutorials>`.

.. rubric:: General Usage

Now that you have a bit more time, we suggest you read about the :ref:`Infrastructure <infrastructure>` to understand how to interact with ``ORK``. You can then go through the different steps of object recognition:

   * :ref:`Data Capture <orkcapture:ork_capture>` ...
   * ... from which you can perform :ref:`Training <training>` of object models ...
   * ... that you can then use for :ref:`Detection <detection>`

.. rubric:: ROS integration

The recognition kitchen was built in a ROS agnostic way, but ROS components were also developed for integration with the ROS ecosystem (e.g. publishers, subscribers, actionlib server, RViz plugin ...). For more info, check out the :ref:`ROS Integration <orkros:ros>`.

.. rubric:: Recognition Pipelines

Several object recognition pipelines have been implemented for this framework. Their documentation is work in progress :) :

+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+
| Techniques                                   | 2D/3D        | Types of object              | Limitations                                                  |
+==============================================+==============+==============================+==============================================================+
| :ref:`LINE-MOD <orklinemod:line_mod>`        | 2D and/or 3D | * rigid, Lambertian          | * does not work with partial occlusions                      |
+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+
| :ref:`tabletop <orktabletop:tabletop>`       | 3D           | * rigid, Lambertian          | * scales linearly with the number of objects                 |
|                                              |              | * rotationally symmetric     | * the object is assumed to be on a table with no 3d rotation |
|                                              |              | * also finds planar surfaces |                                                              |
+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+
| :ref:`TOD <orktod:tod>`                      | 2D and 3D    | * rigid, Lambertian          |                                                              |
|                                              |              | * textured                   |                                                              |
+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+
| :ref:`transparent objects                    | 2D and 3D    | * rigid and transparent      | * Training has to be done on a painted version of the object |
| <orktransparentobjects:transparent_objects>` |              |                              |                                                              |
+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+

.. rubric:: Tools

There are several tools that are used by some of the pipeline and you might need them for your own work or pipelines:

  * :ref:`Reconstruction <orkreconstruction:reconstruction>`

Developers' corner
##################

You like ``ORK`` ? Well you can add any pipeline or database to it. It is fairly simple and modular, just follow the :ref:`Developer Guide <ork_developer>`

Contacts
########

For bug reports and comments, please use the `GitHub infrastructure <https://github.com/wg-perception/>`_ or
join us on the `Google Group <https://groups.google.com/forum/#!forum/object-recognition-kitchen>`_.

If you want to cite this work, please use the BibTeX reference:

.. code-block:: latex

   @misc{ork_ros,
      Author = {Willow Garage, ROS community},
      Title = "{ORK}: {O}bject {R}ecognition {K}itchen},
      howpublished = {\url{https://github.com/wg-perception/object_recognition_core}}
   }
