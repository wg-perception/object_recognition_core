Detection
=========

Sink
----

Detection Pipelines
-------------------
Detection pipelines should implement the :py:class:`object_recognition.pipelines.DetectionPipeline`.

.. autoclass:: object_recognition.pipelines.DetectionPipeline
   :members:

Database
--------

If you are using the predefined ModelWriter, you will also want to use our ModelReader for simplicity.

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
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
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
