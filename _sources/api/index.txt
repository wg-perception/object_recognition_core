DB API
======

DB interactions are done with a specific API. You can either use the default types, or create your own.

.. toctree::
   :maxdepth: 2

   db.rst

Object Recognition API
======================

In the Object Recognition Kitchen each algorithm is defined in terms of a pipeline. We have a few interfaces you should follow if
you wish to add a new pipeline to reduce the amount of code to write.

Our infrastructure is very flexible thanks to ``ecto`` and we have developed several tools to facilitate the traditional
pipelines that contain a training phase and a detection phase. If your pipeline does not require training, just skip
the section.

Once you wrap your code into those ``base classes`` (it's slightly more complex than a base class), you have access to everything
we've coded for you. Doing so usually requires two steps: writing an ``ecto`` cell in pure C++ (or Python if you know a bit of ``ecto``)
which is the core of your processing, and a simple Python wrapper that enables ORK to exploit your cell fully (by being able to find it,
by getting automatic docs from it ...).


.. toctree::
   :maxdepth: 2
   
   generic.rst
   training.rst
   detection.rst
   source_sink.rst
   cells.rst
