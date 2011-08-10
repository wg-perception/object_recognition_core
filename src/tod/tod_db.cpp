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
using object_recognition::db_future::DocumentId;

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
                                    "std::string The collection in which to store the models on the db", "models").required(
            true);
        params.declare<std::string>("db_json_params", "std::string The DB parameters, cf. ObjectDb", "models").required(
            true);
        params.declare<std::string>("object_id", "The object id, to associate this frame with.").required(true);
        params.declare<std::string>("model_json_params", "The parameters used for the model, as JSON.").required(true);
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<cv::Mat>("points", "The 3d position of the points.");
        inputs.declare<cv::Mat>("descriptors", "The descriptors.");
      }

      void
      on_object_id_change(const std::string& id)
      {
        object_id_ = id;
        std::cout << "object_id = " << id << std::endl;
      }

      void
      configure(const tendrils& params, const tendrils& inputs,const tendrils& outputs)
      {
        ecto::spore<std::string> object_id = params["object_id"];
        object_id.set_callback(boost::bind(&TodModelInserter::on_object_id_change, this, _1));
        db_.set_params(params.get<std::string>("db_json_params"));
        collection_models_ = params.get<std::string>("collection_models");
        params_ = params.get<std::string>("model_json_params");
        on_object_id_change(params.get<std::string>("object_id"));
      }

      int
      process(const tendrils& inputs,const tendrils& outputs)
      {
        //if (inputs.get<int> ("trigger") != 'c')
        //  return 0;
        std::cout << "Inserting" << std::endl;

        object_recognition::db_future::Document doc;

        doc.set_attachment<cv::Mat>("descriptors", inputs.get<cv::Mat>("descriptors"));
        doc.set_attachment<cv::Mat>("points", inputs.get<cv::Mat>("points"));
        doc.set_value("object_id", object_id_);
        doc.set_value("model_params", params_);
        std::cout << "Persisting" << std::endl;
        doc.Persist(db_, collection_models_);

        return 0;
      }
      object_recognition::db_future::ObjectDb db_;
      DocumentId object_id_;
      CollectionName collection_models_;
      /** The JSON parameters used to compuet the model */
      std::string params_;
    };
  }
}

ECTO_CELL(tod_db, object_recognition::tod::TodModelInserter, "TodModelInserter", "Insert a TOD model in the db")

ECTO_DEFINE_MODULE(tod_db)
{
}
