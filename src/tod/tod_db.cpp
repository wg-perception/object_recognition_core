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

using object_recognition::db_future::CollectionName;
using object_recognition::db_future::ObjectId;

namespace object_recognition
{
  namespace tod
  {
    /** Class inserting the TOD models in the db
     */
    struct TodModelInserter
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<std::string>("collection_models",
                                    "std::string The collection in which to store the models on the db", "models");
        params.declare<std::string>("db_json_params", "std::string The DB parameters, cf. ObjectDb", "models");
        params.declare<std::string>("object_id", "The object id, to associate this frame with.");
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<cv::Mat>("points", "The 3d position of the points.");
        inputs.declare<cv::Mat>("descriptors", "The descriptors.");
        inputs.declare<int>("trigger", "Capture trigger, 'c' for capture.");
      }

      void
      on_object_id_change(const std::string& id)
      {
        object_id_ = id;
        std::cout << "object_id = " << id << std::endl;
      }

      void
      configure(tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        ecto::spore<std::string> object_id = params.at("object_id");
        object_id.set_callback(boost::bind(&TodModelInserter::on_object_id_change, this, _1));
        db_.set_params(params.get<std::string>("db_json_params"));
        collection_models_ = params.get<std::string>("collection_models");
        on_object_id_change(params.get<std::string>("object_id"));
      }

      int
      process(const tendrils& inputs, tendrils& outputs)
      {
        //if (inputs.get<int> ("trigger") != 'c')
        //  return 0;
        std::cout << "Inserting" << std::endl;

        object_recognition::db_future::Document doc;

        doc.set_attachment<cv::Mat>("descriptors", inputs.get<cv::Mat>("descriptors"));
        doc.set_attachment<cv::Mat>("points", inputs.get<cv::Mat>("points"));
        doc.set_value("object_id", object_id_);

        std::cout << "Persisting" << std::endl;
        doc.Persist(db_, collection_models_);
        return 0;
      }
      object_recognition::db_future::ObjectDb db_;
      ObjectId object_id_;
      CollectionName collection_models_;
    };
  }
}

ECTO_CELL(tod_db, object_recognition::tod::TodModelInserter, "TodModelInserter", "Insert a TOD model in the db")

ECTO_DEFINE_MODULE(tod_db)
{
}
