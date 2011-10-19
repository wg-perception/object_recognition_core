Pipelines
=========

In Object Recognition each algorithm is defined in terms of a pipeline. We have a few interfaces the you should follow if
you wish to add a new pipeline.

Detection Pipelines
-------------------
Detection pipelines should implement the :py:class:`object_recognition.pipelines.DetectionPipeline`.

.. autoclass:: object_recognition.pipelines.DetectionPipeline
   :members:
