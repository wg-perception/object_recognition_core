Training
========


Database Cell
-------------

When creating your training pipeline, you will want to store your model in the database.
By using our interface you will
save a lot of time. Creating such a cell is easy and
it will automatically add the following fields to
the model when persisted:

- the object id
- the model parameters
- the fact that the document is a model, of the type given by the member function ``model_type``


Step 1
^^^^^^

First, you will need to create a cell that implements what is added to the DB document. It has to inherit from
``db::bases::ModelWriterImpl``. An example implementation is:

.. code-block:: cpp
    :linenos:

    struct MyAwesomeModelWriterImpl: public db::bases::ModelWriterImpl
    {
    public:
      // You can define the declare_params and configure functions if needed
      
      // This is the standard ecto cell declare_io
      static void
      declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
      {
        inputs.declare(&MyAwesomeModelWriterImpl::model_part_, "model_part", "A part of the model.");
      }

      // Note how this function has an extra argument: the db::Document
      virtual int
      process(const ecto::tendrils& inputs, const ecto::tendrils& outputs, db::Document& doc)
      {
        doc.set_attachment<ModelPartType>("model_part", *model_part_);
        return ecto::OK;
      }

      // The DB needs to know the type of your model, to differentiate it from other methods
      virtual std::string
      model_type() const
      {
        return "My awesome model";
      }

    private:
      ecto::spore<ModelPartType> model_part_;
    };

The ``process`` function fills your document with everything that makes your model specific. Use the ``set_value`` to add fields that can be searched (strings, numerals) and the ``set_attachment``
member function of the ``db::Document`` to fill the different binary blobs of the model. If your type is not supported, you
will have to implement the following member functions (and please send us a patch containing them if it is a very common type):

.. code-block:: cpp
    :linenos:

    template<>
    void
    Document::get_attachment<YourType>(const AttachmentName &attachment_name, YourType & value) const;

    template<>
    void
    Document::get_attachment_and_cache<YourType>(const AttachmentName &attachment_name, YourType & value);

    template<>
    void
    Document::set_attachment<YourType>(const AttachmentName &attachment_name, const cv::Mat & value);

(we provide a default ``boost`` serialization but to make sure the binary blobs are readable on machines with a different
``boost serialization`` library, you should probably implement your own serialization)

Step 2
^^^^^^

Very important, you need to actually define a type for your cell based on ModelWriterBase. Something like this suffices:

.. code-block:: cpp

    typedef db::bases::ModelWriterBase<MyAwesomeModelWriterImpl> MyAwesomeModelWriterCell;

Python Plasm
------------

Your pipeline also has to provide an implementation of the :py:class:`object_recognition_core.pipelines.training.TrainingPipeline` class so that
``ORK`` can find it (as it will be on the ``PYTHONPATH``), generate docs for it and do everything that is basically not the raw computation.

.. autoclass:: object_recognition_core.pipelines.training.TrainerBase
   :members:
