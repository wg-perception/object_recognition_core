#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <objcog/db/couch.hpp>
#include <objcog/db/opencv.h>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <string>
using ecto::tendrils;
namespace objcog
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
        inputs.declare<int>("trigger", "Capture trigger, 'c' for capture.", 'c');
      }
      ObservationInserter()
          :
            db(std::string(DEFAULT_COUCHDB_URL) + "/frames"),
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
      configure(tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        ecto::spore<std::string> object_id = params.at("object_id");
        object_id.set_callback(boost::bind(&ObservationInserter::on_object_id_change, this, _1));
        ecto::spore<std::string> session_id = params.at("session_id");
        session_id.set_callback(boost::bind(&ObservationInserter::on_session_id_change, this, _1));
        db.create();
      }
      int
      process(const tendrils& inputs, tendrils& outputs)
      {
        if (inputs.get<int>("trigger") != 'c' || inputs.get<bool>("found") == false)
          return 0;
        std::cout << "Inserting" << std::endl;
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
        return 0;
      }
      couch::Db db;
      int frame_number;
      std::string object_id, session_id;
    };
#define STRINGYFY(A) #A

    std::string where_doc_id = STRINGYFY(
        function(doc)
        {
          if(doc.object_id == "%s" )
          emit(null,null);
        }
    );
    struct ObservationReader
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<std::string>("object_id", "The object id, to associate this frame with.", "object_01");
      }

      static void
      declare_io(const tendrils& params, const tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<cv::Mat>("image", "An rgb full frame image.");
        outputs.declare<cv::Mat>("depth", "The 16bit depth image.");
        outputs.declare<cv::Mat>("mask", "The mask.");
        outputs.declare<cv::Mat>("R", "The orientation.");
        outputs.declare<cv::Mat>("T", "The translation.");
        outputs.declare<cv::Mat>("K", "The camera intrinsic matrix");
        outputs.declare<int>("frame_number", "The frame number");
      }
      void
      on_object_id_change(const std::string& id)
      {
        SHOW();
        std::cout << "object_id = " << id << std::endl;
        couch::View v;
        v.add_map("map", boost::str(boost::format(where_doc_id) % id));
        db.run_view(v, -1, 0, total_rows, offset, docs);
        db.print();
        current_frame = 0;

      }
      ObservationReader()
          :
            db(std::string(DEFAULT_COUCHDB_URL) + "/observations"),
            current_frame(0)
      {
        db.create();
      }
      void
      configure(tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        ecto::spore<std::string> object_id = params.at("object_id");
        object_id.set_callback(boost::bind(&ObservationReader::on_object_id_change, this, _1));
      }
      int
      process(const tendrils& inputs, tendrils& outputs)
      {
        couch::Document doc = docs[current_frame];
        doc.update();
        obs << doc; //read the observation from the doc.
        obs >> outputs; //push the observation to the outputs.
        current_frame++;
        if (current_frame >= int(docs.size()))
          return ecto::QUIT;
        return 0;
      }
      std::vector<couch::Document> docs;
      Observation obs;
      int total_rows, offset;
      couch::Db db;
      int current_frame;
    };
  }
}
ECTO_CELL(capture, objcog::capture::ObservationReader, "ObservationReader", "Reads observations from the database.");
