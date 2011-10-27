#include <string>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <ecto/ecto.hpp>

#include "object_recognition/db/db.h"

namespace object_recognition
{
  namespace db
  {
  namespace bases
  {

    struct ModelInserterUtils
    {
      static void declare_params(ecto::tendrils& params);
      static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs);
      static Document populate_doc(const ObjectDb& db,const ObjectId& object_id,const std::string& model_params, const std::string& model_type);
    };
    /** Class inserting the arbitrary Models into the DB
     */
    template<typename T>
    struct ModelInserter
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
        //create a document, and initialize all the common bits.
        object_recognition::db::Document doc(db_, *collection_name_);
        doc.set_value("object_id", *object_id_);
        // Convert the parameters to a property tree and insert them
        boost::property_tree::ptree params;
        std::stringstream ssparams;
        ssparams << model_params_;
        boost::property_tree::read_json(ssparams, params);
        if (params.find("type") != params.not_found())
          params.erase("type");
        doc.set_values("parameters", params);
        doc.set_value("Type", "Model");
        doc.set_value("ModelType", T::ModelType);
        T::process(inputs, outputs, doc);
        doc.Persist();
        return ecto::OK;
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
