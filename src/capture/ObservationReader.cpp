#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <string>

#include <object_recognition/db/db.h>
#include <object_recognition/db/opencv.h>
#include <object_recognition/db/utils.h>
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

    struct ObservationReader
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<std::string>("collection", "The collection to load from.", "observations");
        params.declare<std::string>("db_url", "The database url", std::string(DEFAULT_COUCHDB_URL));
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<std::string>("observation", "The observation id to load.");
        Observation::declare(outputs, false); //not required
        outputs.declare<int>("frame_number", "The frame id number.", 0);
      }
      ObservationReader()
          :
            current_frame(0)
      {
      }
      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        params["db_url"] >> db_url;
        params["collection"] >> collection;
        observation = inputs["observation"];
        db = ObjectDb(db_future::parameters::CouchDB(db_url));
      }
      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        Document doc(db, collection, *observation);
        Observation obs;
        obs << doc;
        obs >> outputs;
        return 0;
      }
      int total_rows, offset;
      ecto::spore<std::string> observation;
      std::string db_url, collection;
      ObjectDb db;
      int current_frame;
    };
  }
}
ECTO_CELL(capture, object_recognition::capture::ObservationReader, "ObservationReader",
          "Reads observations from the database.");
