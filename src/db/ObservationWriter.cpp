#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>
#include <string>

#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/opencv.h>
#include <object_recognition_core/db/prototypes/observations.hpp>

using ecto::tendrils;

namespace object_recognition_core
{
  namespace prototypes
  {
    using db::Document;

    struct ObservationInserter
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare < std::string > ("object_id", "The object id, to associate this frame with.").required(true);
        params.declare < std::string > ("session_id", "The session id, to associate this frame with.").required(true);

        params.declare(&ObservationInserter::db_params_, "db_params", "The database parameters");
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
        object_id = id;
      }
      void
      on_session_id_change(const std::string& id)
      {
        session_id = id;
        frame_number = 0;
      }
      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        db = db_params_->generateDb();
        ecto::spore < std::string > object_id = params["object_id"];
        object_id.set_callback(boost::bind(&ObservationInserter::on_object_id_change, this, _1));
        ecto::spore < std::string > session_id = params["session_id"];
        session_id.set_callback(boost::bind(&ObservationInserter::on_session_id_change, this, _1));
      }
      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        Observation obs;
        obs << inputs;
        // Use the frame_number tendril if provided
        if (inputs.find("frame_number")->second->user_supplied())
          frame_number = inputs.get<int>("frame_number");

        std::cout << "Inserting frame: " << frame_number << std::endl;
        obs.frame_number = frame_number++;

        obs.object_id = object_id;
        obs.session_id = session_id;
        Document doc;
        doc.set_db(db);
        obs >> &doc;
        doc.Persist();
        return ecto::OK;
      }
      int frame_number;
      std::string object_id, session_id;
      ecto::spore<db::ObjectDbParameters> db_params_;
      db::ObjectDbPtr db;
    };
  }
}

ECTO_CELL(db, object_recognition_core::prototypes::ObservationInserter, "ObservationInserter",
    "Inserts observations into the database.")
