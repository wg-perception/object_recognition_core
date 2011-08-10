#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <string>

#include "object_recognition/capture/capture.hpp"
#include "object_recognition/db/couch.hpp"
#include "object_recognition/db/opencv.h"

using ecto::tendrils;
namespace object_recognition
{
  namespace capture
  {
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
        inputs.declare<cv::Mat>("image", "An rgb full frame image.");
        inputs.declare<cv::Mat>("depth", "The 16bit depth image.");
        inputs.declare<cv::Mat>("mask", "The mask.");
        inputs.declare<cv::Mat>("R", "The orientation.");
        inputs.declare<cv::Mat>("T", "The translation.");
        inputs.declare<cv::Mat>("K", "The camera intrinsic matrix");
        inputs.declare<bool>("found", "Whether or not the R|T is valid.", false);
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
      configure(const tendrils& params, const tendrils& inputs,const tendrils& outputs)
      {
        db = couch::Db(params.get<std::string>("db_url") + "/observations");
        db.create();
        ecto::spore<std::string> object_id = params["object_id"];
        object_id.set_callback(boost::bind(&ObservationInserter::on_object_id_change, this, _1));
        ecto::spore<std::string> session_id = params["session_id"];
        session_id.set_callback(boost::bind(&ObservationInserter::on_session_id_change, this, _1));
      }
      int
      process(const tendrils& inputs,const tendrils& outputs)
      {
        if (inputs.get<bool>("found") == false)
          return 0;
        std::cout << "Inserting frame: " << frame_number << std::endl;
        Observation obj;
        obj.image = inputs.get<cv::Mat>("image");
        obj.depth = inputs.get<cv::Mat>("depth");
        if (obj.depth.depth() == CV_32F)
        {
          obj.depth.clone().convertTo(obj.depth, CV_16UC1, 1000);
        }
        obj.mask = inputs.get<cv::Mat>("mask");
        obj.R = inputs.get<cv::Mat>("R");
        obj.T = inputs.get<cv::Mat>("T");
        obj.K = inputs.get<cv::Mat>("K");
        obj.frame_number = frame_number;
        obj.object_id = object_id;
        obj.session_id = session_id;
        couch::Document doc(db);
        doc.create();
        obj >> doc;
        frame_number++;
        return ecto::OK;
      }
      couch::Db db;
      int frame_number;
      std::string object_id, session_id;
    };
  }
}
ECTO_CELL(capture, object_recognition::capture::ObservationInserter, "ObservationInserter",
          "Inserts observations into the database.");
