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

#define DEFAULT_COUCHDB_URL "http://localhost:5984"

using ecto::tendrils;
namespace object_recognition
{
  namespace capture
  {

//    struct result_less
//    {
//      bool
//      operator()(const couch::View::result& lhs, const couch::View::result& rhs)
//      {
//        //these need to be in this ordering for chronological reasons.
//        return boost::lexical_cast<int>(lhs.value) < boost::lexical_cast<int>(rhs.value);
//      }
//    };

    struct ObservationReader
    {
      static void
      declare_params(tendrils& params)
      {
        params.declare<std::string>("object_id", "The object id, to associate this frame with.");
        params.declare<std::string>("session_id", "The object id, to associate this frame with.");
        params.declare<std::string>("db_url", "The database url", std::string(DEFAULT_COUCHDB_URL));
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
      on_object_id_change(const std::string&)
      {
        std::string id, session;
        id = *object_id;
        session = *session_id;
        std::string view;
        if (!session.empty())
          view = db_future::couch::WhereSessionId(session);
        else if (!id.empty())
          view = db_future::couch::WhereDocId(id);
        else
          throw std::runtime_error("You must supply either an object_id or a session_id.");
//        couch::View v;
//        v.add_map("map", view);
//        std::vector<couch::View::result> results = db.run_view(v, -1, 0, total_rows, offset);
//        std::sort(results.begin(), results.end(), result_less());
//        BOOST_FOREACH(const couch::View::result& x, results)
//            {
//              couch::Document doc(db, x.id);
//              docs.push_back(doc);
//            }
        current_frame = 0;
      }
      ObservationReader()
          :
            current_frame(0)
      {
      }
      void
      configure(const tendrils& params, const tendrils& inputs,const tendrils& outputs)
      {
//        db = couch::Db(params.get<std::string>("db_url") + "/observations");
//        db.update_info();
//        object_id = params["object_id"];
//        session_id = params["session_id"];
//        session_id.set_callback(boost::bind(&ObservationReader::on_object_id_change, this, _1));
//        {
//          // Make sure we make the query even though session is empty
//          std::string session = *session_id;
//          if (session.empty())
//            on_object_id_change("");
//        }
      }
      int
      process(const tendrils& inputs,const tendrils& outputs)
      {
        if (docs.empty()) {
          std::cerr << "No object found with id " << *object_id << std::endl;
          return ecto::QUIT;
        }
        db_future::Document doc = docs[current_frame];
//        doc.update();
//        obs << doc; //read the observation from the doc.
//        obs >> outputs; //push the observation to the outputs.
//        current_frame++;
//        if (current_frame >= int(docs.size()))
//          return ecto::QUIT;
        return 0;
      }
      std::vector<db_future::Document> docs;
//      Observation obs;
      int total_rows, offset;
      ecto::spore<std::string> object_id,session_id;
//      couch::Db db;
      int current_frame;
    };
  }
}
ECTO_CELL(capture, object_recognition::capture::ObservationReader, "ObservationReader", "Reads observations from the database.");
