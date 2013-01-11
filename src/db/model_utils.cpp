#include <algorithm>

#include <object_recognition_core/common/json_spirit/json_spirit_reader_template.h>
#include <object_recognition_core/common/json.hpp>
#include <object_recognition_core/db/model_utils.h>
#include <object_recognition_core/db/view.h>

namespace
{
  /** Function comparing two JSON spirit arrays
   * @param obj1
   * @param obj2
   * @return true if the two arrays contain the same elements
   */
  bool
  CompareJsonArrays(const or_json::mArray &obj1, const or_json::mArray &obj2)
  {
    if (obj1.size() != obj2.size())
      return false;
    or_json::mArray::const_iterator iter1 = obj1.begin(), iter2 = obj2.begin();
    for (; iter1 != obj1.end(); ++iter1, ++iter2)
      if (!object_recognition_core::db::CompareJsonIntersection(*iter1, *iter2))
        return false;
    return true;
  }

  /** Function comparing two JSON spirit objects
   * @param obj1
   * @param obj2
   * @return true if the two arrays contain the same elements
   */
  bool
  CompareJsonObjects(const or_json::mObject &obj1, const or_json::mObject &obj2)
  {
    // Go over each key of one
    BOOST_FOREACH(or_json::mObject::const_reference val, obj1)
    {
      // Don't do anything if the value is not present
      or_json::mObject::const_iterator iter = obj2.find(val.first);
      if (iter == obj2.end())
        continue;
      if (!object_recognition_core::db::CompareJsonIntersection(val.second, iter->second))
        return false;
    }
    return true;
  }
}

namespace object_recognition_core
{
  namespace db
  {
    bool
    CompareJsonIntersection(const or_json::mValue &val1, const or_json::mValue &val2)
    {
      // If the type is different, we are done
      if (val1.type() != val2.type())
        return false;
      bool is_same;
      switch (val1.type())
      {
        case or_json::obj_type:
          is_same = CompareJsonObjects(val1.get_obj(), val2.get_obj());
          break;
        case or_json::array_type:
          is_same = CompareJsonArrays(val1.get_array(), val2.get_array());
          break;
        default:
          is_same = val1 == val2;
          break;
      }

      return is_same;
    }

    Document
    PopulateDoc(const ObjectDbPtr& db, const ObjectId& object_id, const std::string& session_ids,
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
      or_json::mValue submethod = to_json(submethod_str);
      or_json::mValue parameters = to_json(parameters_str);
      or_json::mValue sessions = to_json(session_ids);

      doc.set_value("session_ids", sessions);
      doc.set_value("Type", "Model");
      doc.set_value("method", method);
      doc.set_value("submethod", submethod);
      doc.set_value("parameters", parameters);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Documents
    ModelDocuments(ObjectDbPtr &db, const std::vector<ObjectId> & object_ids, const std::string & method,
                   const std::string & json_submethod)
    {
      Documents model_documents;
      model_documents.reserve(object_ids.size());

      // ext, for each object id, find the models (if any) that fit the parameters
      or_json::mValue submethod;
      or_json::read(json_submethod, submethod);

      BOOST_FOREACH(const ModelId & object_id, object_ids)
      {
        View view(View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE);
        view.Initialize(method);
        view.set_key(object_id);
        ViewIterator view_iterator = ViewIterator(view, db).begin();

        while (view_iterator != ViewIterator::end())
        {
          const or_json::mObject & obj = (*view_iterator).fields();
          // Compare the parameters to the input ones
          if (CompareJsonIntersection(submethod, obj.find("submethod")->second))
            model_documents.push_back(Document(db, obj.find("_id")->second.get_str()));

          ++view_iterator;
        }
      }
      return model_documents;
    }
  }
}
