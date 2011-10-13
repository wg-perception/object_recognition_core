#include <string>

#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <opencv2/core/core.hpp>

#include <ecto/ecto.hpp>

#include "object_recognition/db/db.h"
#include "object_recognition/db/opencv.h"

using ecto::tendrils;

using object_recognition::db_future::CollectionName;

typedef std::string ModelId;

namespace object_recognition
{
  namespace tod
  {
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** Cell that loads a TOD model from the DB
     */
    struct ModelReader
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<std::string>("collection", "The collection where the models are stored.", "models");
        params.declare<db_future::ObjectDbParameters>("db_params", "The DB parameters").required(true);
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        inputs.declare<std::string>("model_id", "The DB id of the model to load.");
        outputs.declare<cv::Mat>("descriptors", "The descriptors.");
        outputs.declare<std::string>("object_id", "The DB object ID.");
        outputs.declare<cv::Mat>("points", "The 3d position of the points.");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        db_params_ = params["db_params"];
        collection_ = params["collection"];

        db_ = db_future::ObjectDb(*db_params_);
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        const std::string & model_id = inputs.get<std::string>("model_id");
        db_future::Document doc(db_, *collection_, model_id);

        cv::Mat points, descriptors;
        doc.get_attachment<cv::Mat>("points", points);
        doc.get_attachment<cv::Mat>("descriptors", descriptors);

        outputs["descriptors"] << descriptors;
        outputs["object_id"] << doc.id();
        outputs["points"] << points;

        return ecto::OK;
      }
      object_recognition::db_future::ObjectDb db_;
      ecto::spore<CollectionName> collection_;
      ecto::spore<db_future::ObjectDbParameters> db_params_;
    };
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** Cell that loads a TOD model from the DB
     */
    struct ModelReaderIterative
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<boost::python::object>("model_ids", "The DB id of the model to load.");
        params.declare<std::string>("collection", "The collection where the models are stored.", "models");
        params.declare<db_future::ObjectDbParameters>("db_params", "The DB parameters").required(true);
      }

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<std::vector<cv::Mat> >("points", "The 3d position of the points.");
        outputs.declare<std::vector<cv::Mat> >("descriptors", "The descriptors.");
      }

      void
      configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        db_params_ = params["db_params"];
        collection_ = params["collection"];
        db_ = db_future::ObjectDb(*db_params_);

        const boost::python::object & python_object_ids = params.get<boost::python::object>("object_ids");
        boost::python::stl_input_iterator<std::string> begin(python_object_ids), end;
        std::copy(begin, end, std::back_inserter(model_ids_));
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        // Load the list of models to load

        std::vector<cv::Mat> point_vector, descriptor_vector;
        std::vector<std::string> object_ids;

        BOOST_FOREACH(const ModelId & model_id, model_ids_)
            {
              db_future::Document doc(db_, *collection_, model_id);

              cv::Mat descriptors, points;
              doc.get_attachment<cv::Mat>("descriptors", descriptors);
              doc.get_attachment<cv::Mat>("points", points);

              descriptor_vector.push_back(descriptors);
              object_ids.push_back(doc.id());
              point_vector.push_back(points);
            }

        outputs["descriptors"] << descriptor_vector;
        outputs["object_ids"] << object_ids;
        outputs["points"] << point_vector;

        return ecto::OK;
      }
      object_recognition::db_future::ObjectDb db_;
      ecto::spore<CollectionName> collection_;
      ecto::spore<db_future::ObjectDbParameters> db_params_;
      std::vector<ModelId> model_ids_;
    };
  }
}

ECTO_CELL(tod_detection, object_recognition::tod::ModelReader, "ModelReader", "Reads a TOD model from the db")
ECTO_CELL(tod_detection, object_recognition::tod::ModelReaderIterative, "ModelReaderIterative",
          "Reads several TOD models from the db")
