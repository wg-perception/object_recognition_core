#include <boost/python.hpp>

namespace object_recognition_core
{
  namespace db
  {
    void
    wrap_db_pose_result();
  }
}

BOOST_PYTHON_MODULE(interface)
{
  using namespace object_recognition_core::db;
  wrap_db_pose_result();
}
