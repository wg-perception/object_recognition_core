DB API
======

DB interactions are done with a specific API. You can either use the default types, or create your own.

.. toctree::
   :maxdepth: 2

   db.rst

Object Recognition API
======================

In Object Recognition each algorithm is defined in terms of a pipeline. We have a few interfaces you should follow if
you wish to add a new pipeline to reduce the amount of code to write.

Our infrastructure is very flexible thanks to ecto and we have developed several tools to facilitate the traditional
pipelines that contain a training phase and a detection phase. If your pipeline does not require training, just skip
the section.

.. toctree::
   :maxdepth: 2
   
   training.rst
   detection.rst
   cells.rst
