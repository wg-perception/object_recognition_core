#include <algorithm>

#include "object_recognition/common/json_spirit/json_spirit_reader_template.h"
#include "object_recognition/db/ModelInserter.hpp"

namespace
{
  // Helper functions to compare JSON arrays
  bool
  CompareJsonArrays(const json_spirit::mArray &obj1, const json_spirit::mArray &obj2)
  {
    if (obj1.size() != obj2.size())
      return false;
    return std::equal(obj1.begin(), obj1.end(), obj2.begin());
  }
}

namespace object_recognition
{
  namespace db
  {
    namespace bases
    {
      /** Function that compares the intersection of two JSON trees
       * @param obj1
       * @param obj2
       * @return true if the intersection between the keys have the same values
       */
      bool
      CompareJsonIntersection(const json_spirit::mObject &obj1, const json_spirit::mObject &obj2)
      {
        // Go over each key of one
        BOOST_FOREACH(json_spirit::mObject::const_reference val, obj1)
            {
              // Don't do anything if the value is not present
              json_spirit::mObject::const_iterator iter = obj2.find(val.first);
              if (iter == obj2.end())
                continue;
              bool is_same;
              switch (iter->second.type())
              {
                case json_spirit::array_type:
                  is_same = CompareJsonArrays(val.second.get_array(), iter->second.get_array());
                  break;
                case json_spirit::obj_type:
                  is_same = CompareJsonIntersection(val.second.get_obj(), iter->second.get_obj());
                  break;
                default:
                  is_same = val.second.get_obj() == iter->second.get_obj();
                  break;
              }
              if (!is_same)
                return false;
            }
        return true;
      }

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
