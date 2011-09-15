#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <string>

#include <object_recognition/db/db.h>
#include <object_recognition/db/opencv.h>

#include <object_recognition/db/models/observations.hpp>

using ecto::tendrils;

namespace object_recognition
{
  namespace capture
  {
    void
    Observation::declare(ecto::tendrils& t, bool required)
    {
      t.declare<cv::Mat>("image", "An rgb full frame image.").required(true);
      t.declare<cv::Mat>("depth", "The 16bit depth image.").required(true);
      t.declare<cv::Mat>("mask", "The mask.").required(true);
      t.declare<cv::Mat>("R", "The orientation.").required(true);
      t.declare<cv::Mat>("T", "The translation.").required(true);
      t.declare<cv::Mat>("K", "The camera intrinsic matrix").required(true);
    }
    void
    operator>>(Observation& o, db_future::Document& doc)
    {
      std::map<std::string, cv::Mat> intrinsics, extrinsics;
      intrinsics["K"] = o.K;
      extrinsics["R"] = o.R;
      extrinsics["T"] = o.T;
      std::stringstream intr_ss, extr_ss;
      object_recognition::db::mats2yaml(intrinsics, intr_ss);
      object_recognition::db::mats2yaml(extrinsics, extr_ss);

      object_recognition::db::png_attach(o.image, doc, "image");
      object_recognition::db::png_attach(o.depth, doc, "depth");
      object_recognition::db::png_attach(o.mask, doc, "mask");
      doc.set_attachment_stream("intrinsics.yml", intr_ss, "text/x-yaml");
      doc.set_attachment_stream("extrinsics.yml", extr_ss, "text/x-yaml");
      doc.set_value("object_id", o.object_id);
      doc.set_value("session_id", o.session_id);
      doc.set_value("frame_number", o.frame_number);
    }

    void
    operator<<(Observation& o, db_future::Document& doc)
    {
      o.object_id = doc.get_value<std::string>("object_id");
      o.session_id = doc.get_value<std::string>("session_id");
      o.frame_number = doc.get_value<int>("frame_number");
      object_recognition::db::get_png_attachment(o.image, doc, "image");
      object_recognition::db::get_png_attachment(o.depth, doc, "depth");
      object_recognition::db::get_png_attachment(o.mask, doc, "mask");
      std::stringstream intr_ss, extr_ss;
      doc.get_attachment_stream("intrinsics.yml", intr_ss);
      doc.get_attachment_stream("extrinsics.yml", extr_ss);
      std::map<std::string, cv::Mat> intrinsics, extrinsics;
      intrinsics["K"] = cv::Mat();
      extrinsics["R"] = cv::Mat();
      extrinsics["T"] = cv::Mat();
      object_recognition::db::yaml2mats(intrinsics, intr_ss);
      object_recognition::db::yaml2mats(extrinsics, extr_ss);
      o.K = intrinsics["K"];
      o.R = extrinsics["R"];
      o.T = extrinsics["T"];
    }
    void
    operator>>(Observation& obs, const ecto::tendrils& o)
    {
      o.get<cv::Mat>("image") = obs.image;
      o.get<cv::Mat>("mask") = obs.mask;
      o.get<cv::Mat>("depth") = obs.depth;
      o.get<cv::Mat>("R") = obs.R;
      o.get<cv::Mat>("T") = obs.T;
      o.get<cv::Mat>("K") = obs.K;
      o.get<int>("frame_number") = obs.frame_number;
    }
    void
    operator<<(Observation& obs, const ecto::tendrils& i)
    {
      i["image"] >> obs.image;
      i["mask"] >> obs.mask;
      i["depth"] >> obs.depth;
      i["R"] >> obs.R;
      i["T"] >> obs.T;
      i["K"] >> obs.K;
      i["frame_number"] >> obs.frame_number;
    }

  }
}
