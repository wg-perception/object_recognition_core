#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <string>

#include "object_recognition/common/types.h"
#include <object_recognition/db/db.h>
#include <object_recognition/db/opencv.h>
#include <object_recognition/db/prototypes/observations.hpp>

#include "object_recognition/db/db.h"

#define DEFAULT_COUCHDB_URL "http://localhost:5984"
using ecto::tendrils;
namespace object_recognition
{
  namespace prototypes
  {

    using db::Document;
    using db::ObjectDb;

    struct ObservationReader
    {
      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare(&ObservationReader::observation_, "document", "The observation id to load.");
        Observation::declare(outputs, false); //not required
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        Observation obs;
        obs << *observation_;
        obs >> outputs;
        return 0;
      }
      ecto::spore<db::Document> observation_;
    };
  }
}
ECTO_CELL(object_recognition_db, object_recognition::prototypes::ObservationReader, "ObservationReader",
          "Reads observations from the database.");
