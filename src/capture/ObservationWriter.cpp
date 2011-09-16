#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>
#include <string>

#include <object_recognition/db/db.h>
#include <object_recognition/db/opencv.h>
#include <object_recognition/db/models/observations.hpp>
#include <object_recognition/db/parameters/couch.hpp>

#define DEFAULT_COUCHDB_URL "http://localhost:5984"
using ecto::tendrils;

namespace object_recognition
{
  namespace capture
  {
    using db_future::Document;
    using db_future::ObjectDb;
    struct ObservationInserter
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<std::string>("object_id", "The object id, to associate this frame with.").required(true);
        params.declare<std::string>("session_id", "The session id, to associate this frame with.").required(true);
        params.declare<std::string>("db_url", "The database url", std::string(DEFAULT_COUCHDB_URL));
      }
      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        Observation::declare(inputs, true); //required
      }
      ObservationInserter()
          :
            frame_number(0)
      {
      }
      void
      on_object_id_change(const std::string& id)
      {
        std::cout << "object_id = " << id << std::endl;
        object_id = id;
      }
      void
      on_session_id_change(const std::string& id)
      {
        std::cout << "session_id = " << id << std::endl;
        session_id = id;
        frame_number = 0;
      }
      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        std::string url;
        params["db_url"] >> url;
        db = ObjectDb(db_future::parameters::CouchDB(url));
        ecto::spore<std::string> object_id = params["object_id"];
        object_id.set_callback(boost::bind(&ObservationInserter::on_object_id_change, this, _1));
        ecto::spore<std::string> session_id = params["session_id"];
        session_id.set_callback(boost::bind(&ObservationInserter::on_session_id_change, this, _1));
      }
      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        std::cout << "Inserting frame: " << frame_number << std::endl;
        Observation obs;
        obs << inputs;
        obs.frame_number = frame_number++;
        obs.object_id = object_id;
        obs.session_id = session_id;
        Document doc(db, "observations");
        obs >> doc;
        doc.Persist();
        return ecto::OK;
      }
      int frame_number;
      std::string object_id, session_id;
      db_future::ObjectDb db;
    };
  }
}
ECTO_CELL(capture, object_recognition::capture::ObservationInserter, "ObservationInserter",
          "Inserts observations into the database.");
