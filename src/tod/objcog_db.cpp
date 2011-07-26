#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <string>

#include "object_recognition/db/couch.hpp"
#include "object_recognition/db/opencv.h"

using ecto::tendrils;

namespace object_recognition
{
namespace tod
{
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
  static void declare_params(tendrils& params)
  {
    params.declare<std::string>("object_id", "The object id, to associate this frame with.", "object_01");
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Point3f>("points", "The 3d position of the points.");
    inputs.declare<cv::Mat>("descriptors", "The descriptors.");
    inputs.declare<int>("trigger", "Capture trigger, 'c' for capture.");
  }

  TodModelReader() :
      db_(std::string(DEFAULT_COUCHDB_URL) + "/model")
  {
    db_.create();
  }

  void on_object_id_change(const std::string& id)
  {
//      SHOW();
//      std::cout << "object_id = " << id << std::endl;
//      couch::View v;
//      //v.add_map("map", boost::str(boost::format(where_doc_id) % id));
//      db_.run_view(v, -1, 0, total_rows_, offset_, docs_);
//      db_.print();
//      current_frame_ = 0;
  }

  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    ecto::spore<std::string> object_id = params.at("object_id");
    object_id.set_callback(boost::bind(&TodModelReader::on_object_id_change, this, _1));
    db_.create();
    on_object_id_change(params.get<std::string>("object_id"));
  }

  int process(const tendrils& inputs, tendrils& outputs)
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
  static void declare_params(tendrils& params)
  {
    params.declare<std::string>("object_id", "The object id, to associate this frame with.", "object_01");
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<cv::Mat>("points", "The 3d position of the points.");
    inputs.declare<cv::Mat>("descriptors", "The descriptors.");
    inputs.declare<int>("trigger", "Capture trigger, 'c' for capture.");
  }

  TodModelInserter() :
      db_(std::string(DEFAULT_COUCHDB_URL) + "/model")
  {
    db_.create();
  }
  void on_object_id_change(const std::string& id)
  {
    SHOW();
    object_id_ = id;
    std::cout << "object_id = " << id << std::endl;
  }
  void configure(tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    ecto::spore<std::string> object_id = params.at("object_id");
    object_id.set_callback(boost::bind(&TodModelInserter::on_object_id_change, this, _1));
    db_.create();
    on_object_id_change(params.get<std::string>("object_id"));
  }

  int process(const tendrils& inputs, tendrils& outputs)
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
}
}

ECTO_CELL(tod_db, object_recognition::tod::TodModelInserter, "TodModelInserter", "Insert a TOD model in the db")
ECTO_CELL(tod_db, object_recognition::tod::TodModelReader, "TodModelReader", "Read a TOD model in the db")

ECTO_DEFINE_MODULE(tod_db)
{
}
