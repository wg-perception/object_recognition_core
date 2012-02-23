#include <boost/python.hpp>

namespace object_recognition_core
{
  namespace db
  {
    void
    wrap_db_documents();
    void
    wrap_db_models();
    void
    wrap_db_parameters();
  }
}

BOOST_PYTHON_MODULE(interface)
{
  using namespace object_recognition_core::db;
  wrap_db_documents();
  wrap_db_models();
  wrap_db_parameters();
}
