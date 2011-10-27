#include <object_recognition/db/ModelInserter.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
namespace object_recognition
{
  namespace db
  {
    namespace bases
    {

      Document
      ModelInserterUtils::populate_doc(const ObjectDb& db, const CollectionName& collection_name,
                                       const ObjectId& object_id, const std::string& model_params,
                                       const std::string& model_type)
      {
        //create a document, and initialize all the common bits.
        Document doc(db, collection_name);
        doc.set_value("object_id", object_id);
        // Convert the parameters to a property tree and insert them
        boost::property_tree::ptree params;
        std::stringstream ssparams;
        ssparams << model_params;
        boost::property_tree::read_json(ssparams, params);
        if (params.find("type") != params.not_found())
          params.erase("type");
        doc.set_values("parameters", params);
        doc.set_value("Type", "Model");
        doc.set_value("ModelType", model_type);
        return doc;
      }

    }
  }
}
