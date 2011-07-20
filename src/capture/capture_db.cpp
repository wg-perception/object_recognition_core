#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <objcog/db/couch.hpp>
#include <objcog/db/opencv.h>
#include <boost/python/stl_iterator.hpp>
#include <boost/format.hpp>
#include <string>
#include <objcog/capture/capture.hpp>
using ecto::tendrils;

namespace objcog
{
  namespace capture
  {

    void
    Observation::operator>>(couch::Document& doc)
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
    Observation::operator>>(ecto::tendrils& o)
    {
      o.get < cv::Mat > ("image") = image;
      o.get < cv::Mat > ("mask") = mask;
      o.get < cv::Mat > ("depth") = depth;
      o.get < cv::Mat > ("R") = R;
      o.get < cv::Mat > ("T") = T;
      o.get < cv::Mat > ("K") = K;
      o.get<int>("frame_number") = frame_number;
    }
    void
    Observation::operator<<(couch::Document& doc)
    {
      doc.update();
      object_id = doc.get_value < std::string > ("object_id");
      session_id = doc.get_value < std::string > ("session_id");
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
  }
}
