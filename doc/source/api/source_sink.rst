Source/Sink
===========

``Sources`` and ``Sinks`` are the inputs/outputs of any pipelines. ``ORK`` provides a few for you but you can extend them by providing your own.
For both, you now know the drill: you need a Python wrapper that wraps your ``Sourc/Sink`` cell. you just need to implement a Python object inheriting
from some base classes.

For a ``Source``:  :py:class:`object_recognition_core.io.source.SourceBase`:

.. autoclass:: object_recognition_core.io.source.OpenNI
   :members:

And for a ``Sink``: py:class:`object_recognition_core.io.source.SinkBase`:

.. autoclass:: object_recognition_core.io.sink.SinkBase
   :members:
