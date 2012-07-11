#include <boost/python.hpp>

namespace object_recognition_core
{
  namespace common
  {
    void
    wrap_db_pose_result();
  }
}

BOOST_PYTHON_MODULE(common)
{
  using namespace object_recognition_core::common;
  wrap_db_pose_result();
}
