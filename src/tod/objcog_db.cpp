#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <objcog/db/couch.hpp>
#include <objcog/db/opencv.h>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <string>
using ecto::tendrils;

namespace db
{

  struct Observation
  {
    cv::Mat image, depth, mask;
    cv::Mat R, T, K;
    std::string object_id, session_id;
    int frame_number;

    void
    operator>>(couch::Document& doc)
    {
      std::map<std::string, cv::Mat> intrinsics, extrinsics;
      intrinsics["K"] = K;
      extrinsics["R"] = R;
      extrinsics["T"] = T;
      std::stringstream intr_ss, extr_ss;
      objcog::db::mats2yaml(intrinsics, intr_ss);
      objcog::db::mats2yaml(extrinsics, extr_ss);

      doc.update();
      objcog::db::png_attach(image, doc, "image");
      objcog::db::png_attach(depth, doc, "depth");
      objcog::db::png_attach(mask, doc, "mask");
      doc.attach("intrinsics.yml", intr_ss, "text/x-yaml");
      doc.attach("extrinsics.yml", extr_ss, "text/x-yaml");
      doc.update();
      doc.set_value("object_id", object_id);
      doc.set_value("session_id", session_id);
      doc.set_value("frame_number", frame_number);
      doc.commit();
    }

    void
    operator>>(ecto::tendrils& o)
    {
      o.get<cv::Mat>("image") = image;
      o.get<cv::Mat>("mask") = mask;
      o.get<cv::Mat>("depth") = depth;
      o.get<cv::Mat>("R") = R;
      o.get<cv::Mat>("T") = T;
      o.get<cv::Mat>("K") = K;
      o.get<int>("frame_number") = frame_number;
    }
    void
    operator<<(couch::Document& doc)
    {
      doc.update();
      object_id = doc.get_value<std::string>("object_id");
      session_id = doc.get_value<std::string>("session_id");
      frame_number = doc.get_value<int>("frame_number");
      objcog::db::get_png_attachment(image, doc, "image");
      objcog::db::get_png_attachment(depth, doc, "depth");
      objcog::db::get_png_attachment(mask, doc, "mask");
      std::stringstream intr_ss, extr_ss;
      doc.get_attachment_stream("intrinsics.yml", intr_ss);
      doc.get_attachment_stream("extrinsics.yml", extr_ss);
      std::map<std::string, cv::Mat> intrinsics, extrinsics;
      intrinsics["K"] = cv::Mat();
      extrinsics["R"] = cv::Mat();
      extrinsics["T"] = cv::Mat();
      objcog::db::yaml2mats(intrinsics, intr_ss);
      objcog::db::yaml2mats(extrinsics, extr_ss);
      K = intrinsics["K"];
      R = extrinsics["R"];
      T = extrinsics["T"];
    }
  };
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  struct TodModel
  {
    cv::Mat descriptors_;
    std::string object_id_;
    std::vector<cv::Point3f> points_;
  };

  /** Class reading the TOD models in the db
   */
  struct TodModelReader
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("object_id", "The object id, to associate this frame with.", "object_01");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Point3f>("points", "The 3d position of the points.");
      inputs.declare<cv::Mat>("descriptors", "The descriptors.");
      inputs.declare<int>("trigger", "Capture trigger, 'c' for capture.");
    }

    TodModelReader()
        :
          db_(std::string(DEFAULT_COUCHDB_URL) + "/model")
    {
      db_.create();
    }

    void
    on_object_id_change(const std::string& id)
    {
      SHOW();
      std::cout << "object_id = " << id << std::endl;
      couch::View v;
      v.add_map("map", boost::str(boost::format(where_doc_id) % id));
      db_.run_view(v, -1, 0, total_rows_, offset_, docs_);
      db_.print();
      current_frame_ = 0;
    }

    void
    configure(tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      ecto::spore<std::string> object_id = params.at("object_id");
      object_id.set_callback(boost::bind(&TodModelReader::on_object_id_change, this, _1));
      db_.create();
      on_object_id_change(params.get<std::string>("object_id"));
    }

    int
    process(const tendrils& inputs, tendrils& outputs)
    {
      //if (inputs.get<int> ("trigger") != 'c')
      //  return 0;
      couch::Document doc = docs_[current_frame_];
      doc.update();

      //doc.get_attachment<std::vector<cv::Point3f> >("points", outputs.get<std::vector<cv::Point3f> >("points"));
      doc.get_attachment<cv::Mat>("descriptors", outputs.get<cv::Mat>("descriptors"));

      return 0;
    }

    std::vector<couch::Document> docs_;
    std::string object_id_;
    int total_rows_, offset_;
    couch::Db db_;
    int current_frame_;
  };

  /** Class inserting the TOD models in the db
   */
  struct TodModelInserter
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("object_id", "The object id, to associate this frame with.", "object_01");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<cv::Mat>("points", "The 3d position of the points.");
      inputs.declare<cv::Mat>("descriptors", "The descriptors.");
      inputs.declare<int>("trigger", "Capture trigger, 'c' for capture.");
    }

    TodModelInserter()
        :
          db_(std::string(DEFAULT_COUCHDB_URL) + "/model")
    {
      db_.create();
    }
    void
    on_object_id_change(const std::string& id)
    {
      SHOW();
      object_id_ = id;
      std::cout << "object_id = " << id << std::endl;
    }
    void
    configure(tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      ecto::spore<std::string> object_id = params.at("object_id");
      object_id.set_callback(boost::bind(&TodModelInserter::on_object_id_change, this, _1));
      db_.create();
      on_object_id_change(params.get<std::string>("object_id"));
    }

    int
    process(const tendrils& inputs, tendrils& outputs)
    {
      //if (inputs.get<int> ("trigger") != 'c')
      //  return 0;
      std::cout << "Inserting" << std::endl;

      couch::Document doc(db_);
      doc.create();

      doc.attach("descriptors", inputs.get<cv::Mat>("descriptors"));
      doc.attach("points", inputs.get<cv::Mat>("points"));
      doc.set_value("object_id", object_id_);

      doc.commit();

      return 0;
    }
    couch::Db db_;
    std::string object_id_;
  };

  namespace bp = boost::python;
  bool
  insert_object(std::string object_id, std::string object_desc, bp::object tags)
  {
    couch::Db id_db(std::string(DEFAULT_COUCHDB_URL) + "/objects");
    id_db.create();
    bp::stl_input_iterator<std::string> begin(tags), end;
    std::vector<std::string> tags_v;
    std::copy(begin, end, std::back_inserter(tags_v));
    couch::Document doc(id_db, object_id);
    doc.create();
    doc.set_value("object_id", object_id);
    doc.set_value("description", object_desc);
    doc.set_value("tags", tags_v);
    doc.commit();
    return true;
  }

  bool
  insert_session(std::string session_id, std::string object_id, std::string desc, bp::object tags)
  {
    couch::Db id_db(std::string(DEFAULT_COUCHDB_URL) + "/sessions");
    id_db.create();
    bp::stl_input_iterator<std::string> begin(tags), end;
    std::vector<std::string> tags_v;
    std::copy(begin, end, std::back_inserter(tags_v));
    couch::Document doc(id_db, session_id);
    doc.create();
    doc.set_value("session_id", session_id);
    doc.set_value("object_id", object_id);
    doc.set_value("description", desc);
    doc.set_value("tags", tags_v);
    doc.commit();
    return true;
  }

}

BOOST_PYTHON_MODULE(tod_db)
{
  ecto::wrap<db::ObservationInserter>("ObservationInserter");
  ecto::wrap<db::ObservationReader>("ObservationReader");
  ecto::wrap<db::TodModelInserter>("TodModelInserter");
  ecto::wrap<db::TodModelReader>("TodModelReader");
  boost::python::def("insert_object", db::insert_object);
  boost::python::def("insert_session", db::insert_session);

}
;
