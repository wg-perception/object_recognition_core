Object Recognition Kitchen
==========================

.. highlight:: ectosh

The Object Recognition Kitchen (ORK) is a project developed at Willow Garage for object recognition.

There is currently no unique method to perform object recognition. Objects can be textured, non textured, transparent, articulated, etc.
For this reason, the Object Recognition Kitchen was designed to simultaneously run several object recognition techniques while leveraging all the non-vision stuff for
you (like database management and inputs/outputs handling).

``ORK`` is built on top of ecto which is a lightweight hybrid C++/Python framework for organizing computations as directed acyclic graphs: http://ecto.willowgarage.com.
``ORK`` is independent of ROS. It does interact with ROS through publishers/subscribers for the inputs/outputs if you want to though.


.. rubric:: Table of Contents

.. toctree::
   :maxdepth: 1

   install.rst
   user.rst
   index_developer.rst

Contacts
========

For bug reports and comments, please use the github infrastructure at https://github.com/wg-perception/
  
