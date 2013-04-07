#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <string>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/document.h>
#include <object_recognition_core/db/opencv.h>
#include <object_recognition_core/db/prototypes/observations.hpp>

#define DEFAULT_COUCHDB_URL "http://localhost:5984"
using ecto::tendrils;
namespace object_recognition_core
{
  namespace prototypes
  {

    using db::Document;

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
        obs << &(*observation_);
        obs >> outputs;
        return 0;
      }
      ecto::spore<db::Document> observation_;
    };
  }
}
ECTO_CELL(db, object_recognition_core::prototypes::ObservationReader, "ObservationReader",
    "Reads observations from the database.")
