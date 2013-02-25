:orphan:

.. _infrastructure:

ORK for users
#############

object_recognition_core provides an infrastructure for easy development and usage of object recognition pipelines. When
using/developing a new method you created or found, you usually always have to recreate the following:

   * figure out a way to store/query/use your training data
   * figure out a way to store/query/use your models
   * an easy to train your objects
   * deal with where your data is coming from (cameras, ROS rosbag, ROS topics)
   * deal with where your data is going to
   * integrate it with ROS by listening to the right topics and publishing something
   * compare it to other pipelines by having several pipelines running at once

``ORK`` takes care of all that (and more !) by providing base classes and a flexible infrastructure because let's face
it, you want to work on the juicy stuff: the pipeline itself.

If you are a user, the docs are right below.

User Docs
#########
.. toctree::
   :maxdepth: 2
   
   couch.rst
   configuration.rst
