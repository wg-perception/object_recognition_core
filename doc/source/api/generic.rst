Generic Comments
================

``ORK`` is able to run different object recognition pipelines concurrently while using a common infrastructure: data will come in
through a ``Source`` that will then go to a ``Pipeline`` which will output information to a ``Sink``.


Source/Pipeline/Sink
--------------------

Those are usually written in C++ and can come from any library out there. To be usable, they need to be wrapper in a C++ ``ecto`` cell as
``ORK`` is running everything in a big ``ecto graph``. To have this cell foundable and easier to use, it also needs to be wrapped in a Python
object  (there is a base class for any of the types).

Unit Tests
----------

To make sure your ``Source``/``Pipeline``/``Sink`` is compatible with ``ORK``, we provide CMake macros to test your code. Use them as follows:

.. code-block:: cmake

    find_package(object_recognition_core)
    # for a detection pipeline
    object_recognition_core_detection_test(${PATH_TO_YOUR_DETECTION_CONFIG_FILE})
    # for a training pipeline
    object_recognition_core_training_test(${PATH_TO_YOUR_TRAINING_CONFIG_FILE})
    # for a sink
    object_recognition_core_sink_test(${YOUR_SINK_NAME})
    # for a source
    object_recognition_core_source_test(${YOUR_SOURCE_NAME})
    # for a generic Python script you want to test --help on
    object_recognition_core_help_test(${PATH_TO_YOUR_PYTHON_SCRIPT})
