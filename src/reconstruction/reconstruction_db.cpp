#include <string>

#include <boost/format.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "object_recognition/db/db.h"
#include "object_recognition/db/opencv.h"

using ecto::tendrils;

namespace object_recognition
{
  namespace reconstruction
  {
    void
    insert_mesh(const std::string& db_url, const std::string& object_id, const std::string& session_id,
                const std::string& mesh_file, const std::string& surfel_file)
    {
      object_recognition::db_future::ObjectDb db;
      boost::property_tree::ptree db_p;
      db_p.add("type", "CouchDB");
      db_p.add("url", db_url);
      db.set_params(db_p);
      object_recognition::db_future::Document doc;
      std::ifstream mesh_stream(mesh_file.c_str());
      doc.set_attachment_stream("mesh.ply", mesh_stream);
      std::ifstream surfel_stream(surfel_file.c_str());
      doc.set_attachment_stream("surfel.ply", surfel_stream);
      doc.set_value("object_id", object_id);
      doc.set_value("session_id", session_id);
      doc.Persist(db, "meshes");
    }
  }
}

