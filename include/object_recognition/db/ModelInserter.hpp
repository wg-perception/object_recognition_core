#pragma once

#include <ecto/ecto.hpp>
#include <object_recognition/db/db.h>
#include <object_recognition/common/types.h>
namespace object_recognition
{
  namespace db
  {
    namespace bases
    {

      struct ModelInserterUtils
      {
        static Document
        populate_doc(const ObjectDb& db, const CollectionName& collection_name, const ObjectId& object_id,
                     const std::string& model_params, const std::string& model_type);
      };
      /** Class inserting the arbitrary Models into the DB
       */
      template<typename T>
      struct ModelInserter: T
      {
        typedef ModelInserter<T> C;

        static void
        declare_params(ecto::tendrils& params)
        {
          params.declare(&C::collection_name_, "collection", //
                         "std::string The collection in which to store the models on the db", //
                         "object_recognition").required(true);
          params.declare(&C::db_params_, "db_params", //
                         "The DB parameters").required(true);
          params.declare(&C::object_id_, "object_id", //
                         "The object id, to associate this frame with.").required(true);
          params.declare(&C::model_params_, "model_json_params", //
                         "The parameters used for the model, as JSON.").required(true);
          T::declare_params(params);
        }

        static void
        declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
        {
          T::declare_io(params, inputs, outputs);
        }

        void
        configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
        {
          db_.set_params(*db_params_);
          T::configure(params, inputs, outputs);
        }

        int
        process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
        {
          Document doc_new = ModelInserterUtils::populate_doc(db_, *collection_name_, *object_id_, *model_params_,
                                                          T::model_type());

          // Find all the models with the same parameters
          object_recognition::db::DocumentView view;
          //view.AddView(db_.type(), );




          std::cout << "persisting " << doc_new.id() << std::endl;
          int rval = T::process(inputs, outputs, doc_new);
          if (rval == ecto::OK)
            doc_new.Persist();
          std::cout << "done persisting " << doc_new.id() << std::endl;
          return rval;
        }
      private:
        ObjectDb db_;
        ecto::spore<ObjectDbParameters> db_params_;
        ecto::spore<DocumentId> object_id_;
        ecto::spore<CollectionName> collection_name_;
        ecto::spore<std::string> model_params_;
      };
    }
  }
}
