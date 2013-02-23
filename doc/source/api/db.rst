DB API
======

Core Types
----------

.. doxygenclass:: object_recognition_core::db::ObjectDb
    :project: object_recognition_core
    :members:


.. doxygenclass:: object_recognition_core::db::ObjectDbParameters
    :project: object_recognition_core
    :members:


.. doxygenclass:: object_recognition_core::db::Document
    :project: object_recognition_core
    :members:

Implementing your own DB type
-----------------------------

If you want to create your own DB type, look at the examples from the core like CouchDb or filesystem.
You just have to inherit from ``ObjectDb``.
