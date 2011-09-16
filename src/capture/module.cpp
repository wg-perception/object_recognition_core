#include <ecto/ecto.hpp>
#include <boost/python/stl_iterator.hpp>

//#include "object_recognition/db/couch.hpp"

namespace bp = boost::python;

namespace object_recognition
{
  namespace capture
  {
//    namespace bp = boost::python;
//    bool
//    insert_object(std::string object_id, std::string object_desc, bp::object tags)
//    {
//      couch::Db id_db(std::string(DEFAULT_COUCHDB_URL) + "/objects");
//      id_db.create();
//      bp::stl_input_iterator<std::string> begin(tags), end;
//      std::vector<std::string> tags_v;
//      std::copy(begin, end, std::back_inserter(tags_v));
//      couch::Document doc(id_db, object_id);
//      doc.create();
//      doc.set_value("object_id", object_id);
//      doc.set_value("description", object_desc);
//      doc.set_value("tags", tags_v);
//      doc.commit();
//      return true;
//    }
//    bool
//    insert_session(std::string session_id, std::string object_id, std::string desc, bp::object tags)
//    {
//      couch::Db id_db(std::string(DEFAULT_COUCHDB_URL) + "/sessions");
//      id_db.create();
//      bp::stl_input_iterator<std::string> begin(tags), end;
//      std::vector<std::string> tags_v;
//      std::copy(begin, end, std::back_inserter(tags_v));
//      couch::Document doc(id_db, session_id);
//      doc.create();
//      doc.set_value("session_id", session_id);
//      doc.set_value("object_id", object_id);
//      doc.set_value("description", desc);
//      doc.set_value("tags", tags_v);
//      doc.commit();
//      return true;
//    }
  }
}
ECTO_DEFINE_MODULE(capture)
{
//  boost::python::def("insert_object", object_recognition::capture::insert_object);
//  boost::python::def("insert_session", object_recognition::capture::insert_session);
}
