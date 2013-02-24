Detection
=========

To implement a detection pipeline runnable with the object recognition infrastructure you will need a Python script that implements a plasm for your pipeline. To ease the implementation, we describe a simple way way to retrieve models from the database, as well as the output format of the pipeline.

Python Plasm
------------

Your pipeline has to provide an implementation of the :py:class:`object_recognition_core.pipelines.detection.DetectionPipeline` class to be fully integrated with object recognition infrastructure:

.. autoclass:: object_recognition_core.pipelines.detection.DetectorBase
   :members:

By providing so, after parsing the config file, the detection script will be able to find/load you cell when present on the PYTHONPATH.

Database Cell
-------------

If you are using the predefined ModelWriter for training, you will also want to use our ModelReader for simplicity.

Step 1
^^^^^^

The cell is bit different from the ModelWriter as it reads several models at once. It has to inherit from
``db::bases::ModelReaderImpl``. An example implementation is:

.. code-block:: cpp
    :linenos:

    struct MyAwesomeModelReaderImpl: public db::bases::ModelReaderImpl
    {
    public:
      // This function gives you db_documents for each model from which you can extract the information you want and
      // store it locally (maybe in a search structure)
      void
      ParameterCallback(const Documents & db_documents)
      {
        // Stacking the models, or building a search structure ...
      }

      // The next 4 functions are the standard ecto cells ones
      static void
      declare_params(ecto::tendrils& params)
      {
      }

      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
      }

      void
      configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
      }

      virtual int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
      {
        // Doing some awesome matching or just passing the models to the output
        return ecto::OK;
      }
    };

The ``ParameterCallback`` function gets from the model everything that makes it specific. Please refer to ModelWriter
for how to get this data

Step 2
^^^^^^

Very important, you need to actually define a type for your cell based on ModelReaderBase. Something like this suffices:

.. code-block:: cpp

    typedef db::bases::ModelReaderBase<MyAwesomeModelReaderImpl> MyAwesomeModelReaderCell;

Sink
----

You can output results to the console or to a CSV file.

.. toggle_table::
    :arg1: Non-ROS
    :arg2: ROS

.. toggle:: Non-ROS

    That is all you can output to if you don't use ROS.

.. toggle:: ROS

    If you want to use the ROS publisher that outputs object recognition messages, you need to have your recognition pipeline have the following output tendrils:

    object_ids: a vector of Object ids
    Rs: a vector of cv::Mat, each representing the pose rotation of a matching object
    ts: a vector of cv::Mat, each representing the pose translation of a matching object
