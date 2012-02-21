#include <ecto/ecto.hpp>
namespace object_recognition_core
{
  namespace db
  {
    void wrap_db_documents();
    void wrap_db_models();
    void wrap_db_parameters();
  }
}
ECTO_DEFINE_MODULE(object_recognition_core_db)
{
  using namespace object_recognition_core::db;
  wrap_db_documents();
  wrap_db_models();
  wrap_db_parameters();
}
