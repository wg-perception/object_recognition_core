#include <algorithm>

#include "object_recognition/common/json_spirit/json_spirit_reader_template.h"
#include <object_recognition/db/model_utils.h>
#include <object_recognition/common/json.hpp>

namespace
{
  /** Function comparing two JSON arrays
   * @param obj1
   * @param obj2
   * @return true if the two arrays contain the same elements
   */
  bool
  CompareJsonArrays(const or_json::mArray &obj1, const or_json::mArray &obj2)
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
    bool
    CompareJsonIntersection(const or_json::mObject &obj1, const or_json::mObject &obj2)
    {
      // Go over each key of one
      BOOST_FOREACH(or_json::mObject::const_reference val, obj1)
          {
            // Don't do anything if the value is not present
            or_json::mObject::const_iterator iter = obj2.find(val.first);
            if (iter == obj2.end())
              continue;
            // If the type is different, we are done
            if (val.second.type() != iter->second.type())
              return false;
            bool is_same;
            switch (iter->second.type())
            {
              case or_json::obj_type:
                is_same = CompareJsonIntersection(val.second.get_obj(), iter->second.get_obj());
                break;
              case or_json::array_type:
                is_same = CompareJsonArrays(val.second.get_array(), iter->second.get_array());
                break;
              case or_json::str_type:
                is_same = val.second.get_str() == iter->second.get_str();
                break;
              case or_json::bool_type:
                is_same = val.second.get_bool() == iter->second.get_bool();
                break;
              case or_json::int_type:
                is_same = val.second.get_int64() == iter->second.get_int64();
                break;
              case or_json::real_type:
                is_same = val.second.get_real() == iter->second.get_real();
                break;
              default:
                throw std::runtime_error("non JSON spirit type");
                break;
            }

            if (!is_same)
              return false;
          }
      return true;
    }

    Document
    PopulateDoc(const ObjectDb& db, const ObjectId& object_id, const std::string& session_ids,
                const std::string& method, const std::string& submethod_str, const std::string& parameters_str)
    {
      //create a document, and initialize all the common bits.
      Document doc(db);
      PopulateDoc(object_id, session_ids, method, submethod_str, parameters_str, doc);
      return doc;
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void
    PopulateDoc(const ObjectId& object_id, const std::string& session_ids, const std::string& method,
                const std::string& submethod_str, const std::string& parameters_str, Document& doc)
    {
      doc.set_value("object_id", object_id);
      // Convert the parameters to a property tree and insert them
      or_json::mObject submethod = to_json(submethod_str).get_obj();
      or_json::mObject parameters = to_json(parameters_str).get_obj();
      or_json::mValue sessions = to_json(session_ids);

      doc.set_value("session_ids", sessions);
      doc.set_value("Type", "Model");
      doc.set_value("method", method);
      doc.set_value("submethod", submethod);
      doc.set_value("parameters", parameters);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Documents
    ModelDocuments(ObjectDb &db, const std::vector<ObjectId> & object_ids, const std::string & method,
                   const std::string & json_submethod)
    {
      Documents model_documents;
      model_documents.reserve(object_ids.size());

      // ext, for each object id, find the models (if any) that fit the parameters
      or_json::mObject submethod;
      {
        or_json::mValue value;
        or_json::read(json_submethod, value);
        submethod = value.get_obj();
      }

      BOOST_FOREACH(const ModelId & object_id, object_ids)
          {
            View view(View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE);
            view.Initialize(object_id, method);
            ViewIterator view_iterator = ViewIterator(view, db).begin();

            while (view_iterator != ViewIterator::end())
            {
              // Compare the parameters to the input ones
              or_json::mObject db_parameters = (*view_iterator).get_value<or_json::mObject>("submethod");
              if (CompareJsonIntersection(submethod, db_parameters))
                model_documents.push_back(Document(db, (*view_iterator).id()));

              ++view_iterator;
            }
          }
      return model_documents;
    }
  }
}
