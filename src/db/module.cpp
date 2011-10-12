#include <ecto/ecto.hpp>
namespace object_recognition
{
  namespace db
  {
    void wrap_db_options();
  }
}
ECTO_DEFINE_MODULE(object_recognition_db)
{
  using namespace object_recognition::db;
  wrap_db_options();
}
