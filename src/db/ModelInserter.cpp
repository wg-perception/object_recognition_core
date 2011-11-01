#include <object_recognition/db/ModelInserter.hpp>
#include <boost/property_tree/ptree.hpp>

#include "object_recognition/common/json_spirit/json_spirit_reader_template.h"

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
        json_spirit::mObject params;
        {
          json_spirit::mValue value;
          std::stringstream ssparams;
          ssparams << model_params;
          json_spirit::read(ssparams, value);
          params = value.get_obj();
        }

        params.erase("type");
        doc.set_values("parameters", params);
        doc.set_value("Type", "Model");
        doc.set_value("ModelType", model_type);
        return doc;
      }

    }
  }
}
