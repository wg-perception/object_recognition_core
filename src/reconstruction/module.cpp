#include <ecto/ecto.hpp>

namespace object_recognition
{
  namespace reconstruction
  {
    void
    insert_mesh(const std::string& db_url, const std::string& object_id, const std::string& session_id,
                const std::string& mesh_file, const std::string& surfel_file);
  }
}
ECTO_DEFINE_MODULE(reconstruction)
{
  boost::python::def("insert_mesh", object_recognition::reconstruction::insert_mesh);
}
