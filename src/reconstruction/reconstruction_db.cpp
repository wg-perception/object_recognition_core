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
    /** Class inserting the TOD models in the db
     */
    struct MeshInserter
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<std::string>("object_id", "The object id, to associate this frame with.").required(true);
        params.declare<std::string>("session_id", "The object id, to associate this frame with.").required(true);
        params.declare<std::string>("mesh_file", "The mesh ply file.").required(true);
        params.declare<std::string>("surfel_file", "The surfel ply file.").required(true);
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
      }


      void
      configure(tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        object_id = params["object_id"];
        session_id = params["session_id"];
        mesh_file = params["mesh_file"];
        surfel_file = params["surfel_file"];
        boost::property_tree::ptree db_p;
        db_p.add("type","CouchDB");
        db_p.add("url",*db_url);
        db_.set_params("{\"type\": \"CouchDB\", \"url\": \"whatever_url_you_want:whatever_port"}")
      }

      int
      process(const tendrils& inputs, tendrils& outputs)
      {
        //if (inputs.get<int> ("trigger") != 'c')
        //  return 0;
        std::cout << "Inserting" << std::endl;

        object_recognition::db_future::Document doc;
        std::ifstream mesh_stream(mesh_file->c_str());
        doc.set_attachment_stream("mesh.ply",mesh_stream);
        std::ifstream surfel_stream(surfel_file->c_str());
        doc.set_attachment_stream("surfel.ply",surfel_stream);
        doc.set_value("object_id", *object_id);
        doc.set_value("session_id", *session_id);
        doc.Persist(db_,"");

        return 0;
      }
      object_recognition::db_future::ObjectDb db_;
      ecto::spore<std::string> object_id,session_id,mesh_file,surfel_file,db_url;
    };
  }
}

ECTO_CELL(tod_db, object_recognition::reconstruction::MeshInserter, "MeshInserter", "Insert a mesh into the db.")

ECTO_DEFINE_MODULE(tod_db)
{
}
