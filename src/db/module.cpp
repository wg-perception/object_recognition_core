#include <ecto/ecto.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/list.hpp>

#include "object_recognition/db/db.h"
#include <boost/foreach.hpp>

namespace bp = boost::python;

namespace object_recognition
{
  namespace db
  {
//    bp::list
//    run_view(const std::string url, const std::string& collection, const std::string& view)
//    {
////      int total_rows, offset;
////      bp::list docs;
////      couch::View v;
////      v.add_map("map", view);
////      db_future::ObjectDb db(url + "/" + collection);
////      std::vector<couch::View::result> results = db.run_view(v, -1, 0, total_rows, offset);
////      BOOST_FOREACH(const couch::View::result& x, results)
////          {
////            docs.append(bp::make_tuple(x.id, x.key, x.value));
////          }
//      return docs;
//    }
  }
}
ECTO_DEFINE_MODULE(object_recognition_db)
{
//  bp::def("run_view", object_recognition::db::run_view,
//          (bp::arg("url"), bp::arg("collection"), bp::arg("view")));
}
